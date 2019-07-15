/*
 * PathGenerator.cpp
 *
 *  Created on: 12 Jun 2019
 *      Author: julian
 */
#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "spline.h"
#include "PathGenerator.h"
#include "Vehicle.h"
#include "helpers.h"
// uncomment to disable assert()
// #define NDEBUG
#include <cassert>

using std::string;
using std::vector;
using std::cout;
using std::endl;

#define MPH2METPS (1/2.237)
#define UPDATE_INTERVAL 0.02
#define MIN_DISTANCE_TO_OTHERS 40
#define MIN_DISTANCE_TO_OTHERS_OVERTAKING 45
#define TIMER_COUNTDOWN 30 // ~ 5 seconds
#define TOL .5

bool sortByFirstElement(std::pair<double, double> a,
		std::pair<double, double> b) {
	return a.first < b.first;
}

PathGenerator::PathGenerator(vector<landmark_s> &landmarks) :
		landmarks(landmarks) {
	this->landmarks = landmarks;
	reset();
}

PathGenerator::~PathGenerator() {
}

void PathGenerator::reset() {
	current_desired_lane = 1;
	dist_prev = 0;
	dist_prevprev = 0;
	vehicles.clear();
	previous_path_x.clear();
	previous_path_y.clear();
	x_prev = 0;
	target_speed = speed_limit_mph;
	not_yet_changed_lanes = false;
	previous_lane = -1;
	timer = -1;
}

bool PathGenerator::is_in_current_lane(double v_d) {
	return is_near_same_lane(v_d, current_desired_lane);
}

int PathGenerator::get_center_of_lane(int lane_number) {
	return 2 + 4 * lane_number;
}

bool PathGenerator::is_in_target_lane_close_to_center(double v_d) {
	double center_of_target_lane = get_center_of_lane(current_desired_lane);
	return (std::abs(center_of_target_lane - v_d) < TOL);
}

bool PathGenerator::is_almost_in_lane(int lane, double d) {
	// if another vehicle changes lanes (or driver is drunk) it could be almost in our lane...
	double right_lane_boundary = 4 * (lane + 1);
	double left_lane_boundary = 4 * (lane);
	bool close_to_right_boundary = std::abs(right_lane_boundary - d) < TOL;
	bool close_to_left_boundary = std::abs(left_lane_boundary - d) < TOL;

	return close_to_right_boundary || close_to_left_boundary;
}

bool PathGenerator::is_near_same_lane(double other_car_d, int lane_number) {
	int other_car_lane = int(other_car_d / 4.0);
	bool is_close = is_almost_in_lane(lane_number, other_car_d);
	return other_car_lane == lane_number || is_close;
}

void PathGenerator::updateOtherCars(vector<vector<double>> sensor_fusion_data) {
	vehicles.clear();
	for (vector<double> single_data : sensor_fusion_data) {
		int id = int(single_data[0]);
		double x = single_data[1];
		double y = single_data[2];
		double vx = single_data[3];
		double vy = single_data[4];
		double s = single_data[5];
		double d = single_data[6];

		map<int, Vehicle>::iterator lookup = this->vehicles.find(id);
		if (lookup == this->vehicles.end()) {
			Vehicle newVehicle(id, x, y, vx, vy, s, d);
			this->vehicles[id] = newVehicle;
		} else {

			Vehicle veh = this->vehicles.find(id)->second;
			veh.updateVehicle(x, y, vx, vy, s, d);
		}
	}
}

void PathGenerator::update_ego_state(vector<double> previous_path_x,
		vector<double> previous_path_y, double end_path_s, double end_path_d,
		double car_x, double car_y, double car_s, double car_d, double car_yaw,
		double car_speed) {

	this->previous_path_x = previous_path_x;
	this->previous_path_y = previous_path_y;
	this->end_path_s = end_path_s;
	this->end_path_d = end_path_d;
	this->car_x = car_x;
	this->car_y = car_y;
	this->car_s = car_s;

	cout << "car_x, car_y, car_yaw, car_speed = " << car_x << ", " << car_y
			<< ", " << deg2rad(car_yaw) << ", " << car_speed << endl;

	this->car_d = car_d;
	// we're only using rad
	this->car_yaw = deg2rad(car_yaw);
	this->car_speed = car_speed;
}

double PathGenerator::get_next_x(double max_s) {
	double next_x;
	// obey acceleration / jerk constraints.
	// assumes acceleartion is measured as the derivative of two consecutive velocity values (0.02 secs apart)
	// assumes jerk is measured as the derivative of two consecutive acceleartion values
	// and therefore three consecutive velocity values (each pair 0.02 secs apart)
	if (this->dist_prevprev == 0) {
		if (this->dist_prev == 0) {
			// first iteration, obey acceleration of 10 m/s
			// this wouldn't work if car comes to a halt at some point (e.g. traffic lights)
			next_x = std::min(0.02 * 0.02 * accel_max, max_s);
		} else {
			// second iteration
			next_x = x_prev
					+ std::min(dist_prev + 0.02 * 0.02 * accel_max, max_s);
		}
	} else {
		// third or later iteration, jerk comes into play
		double max_x_for_jerk_max = x_prev + 0.02 * 0.02 * 0.02 * jerk_max
				+ 2 * dist_prev + dist_prevprev;
		double max_x_for_accel_max = x_prev + dist_prev
				+ 0.02 * 0.02 * accel_max;

		next_x = std::min(max_x_for_accel_max,
				std::min(max_x_for_jerk_max, x_prev + max_s));
	}
	assert(next_x <= x_prev + max_s);

	// update prev x
	this->dist_prevprev = this->dist_prev;
	this->dist_prev = next_x - x_prev;
	this->x_prev = next_x;

	return next_x;
}

bool PathGenerator::check_lane_safe(int lane_number) {
	double current_path_x = previous_path_x[previous_path_x.size() - 1];
	double current_path_y = previous_path_y[previous_path_y.size() - 1];

	Vehicle v;
	for (auto const& vehicle_map_entry : vehicles) {
		v = vehicle_map_entry.second;
		if (!is_near_same_lane(v.d, lane_number))
			continue;

		if (distance(current_path_x, current_path_y, v.x, v.y)
				< MIN_DISTANCE_TO_OTHERS_OVERTAKING
				&& !(v.s < car_s - 10 && getNorm(v.vx, v.vy) < car_speed - 5)) {
			return false;
		}
	}
	return true;
}

void PathGenerator::reset_timer_if_necessary() {
	if (timer > 0)
		timer--;

	if (timer == 0) {
		// reset blocking lane
		previous_lane = -1;
	}
}

void PathGenerator::check_lane_change_finished() {
	if (not_yet_changed_lanes && is_in_target_lane_close_to_center(car_d)) {
		not_yet_changed_lanes = false;
	}
}

bool PathGenerator::vehicle_is_in_front(const Vehicle& v) {
	return v.s > car_s;
}

bool PathGenerator::vehicle_is_slower(const Vehicle& v) {
	return car_speed > getNorm(v.vx, v.vy);
}

void PathGenerator::trigger_lane_change(int target_lane) {
	cout << "new current lane is " << target_lane << endl;
	previous_lane = current_desired_lane;
	timer = TIMER_COUNTDOWN;
	current_desired_lane = target_lane;
}

void PathGenerator::sort_values(std::vector<double>& spline_X,
		std::vector<double>& spline_Y) {
	std::vector<std::pair<double, double> > spline_anchor;
	for (size_t i = 0; i < spline_X.size(); i++) {
		std::pair<double, double> next_pair = { spline_X[i], spline_Y[i] };
		spline_anchor.push_back(next_pair);
	}
	std::sort(spline_anchor.begin(), spline_anchor.end(), sortByFirstElement);
	for (size_t i = 0; i < spline_X.size(); i++) {
		spline_X[i] = spline_anchor[i].first;
		spline_Y[i] = spline_anchor[i].second;
	}
	for (size_t i = 0; i < spline_X.size() - 1; i++) {
		if (spline_X[i] > spline_X[i + 1]) {
			cout << "X is not sorted at index " << i << " to " << i + 1 << endl;
			cout << "spline_anchor == " << spline_anchor[i].first << ", "
					<< spline_anchor[i + 1].first << endl;
		}
	}
}

void PathGenerator::get_next_vals(vector<double> &next_x_vals,
		vector<double> &next_y_vals) {

	reset_timer_if_necessary();
	int len_previous_path = this->previous_path_x.size();

	std::vector<double> spline_X, spline_Y;

	// origin is the center of coordinate system for this iteration
	double origin_x, origin_y;
	double end_of_prev_path_yaw_rate = car_yaw;
	if (len_previous_path < 2) {
		// first iteration, so current car pos is origin
		// TODO do we really need this??
		//spline_X.push_back(car_x - cos(car_yaw));
		//spline_Y.push_back(car_y - sin(car_yaw));

		spline_X.push_back(car_x);
		spline_Y.push_back(car_y);

		origin_x = car_x;
		origin_y = car_y;
	} else {
		// already have points from previous path -> use them for smooth lane curvature
		spline_X.push_back(previous_path_x[len_previous_path - 5]);
		spline_Y.push_back(previous_path_y[len_previous_path - 5]);

		spline_X.push_back(previous_path_x[len_previous_path - 1]);
		spline_Y.push_back(previous_path_y[len_previous_path - 1]);

		// calculate angle of last two points, this is the future heading of the car
		end_of_prev_path_yaw_rate = atan2(
				previous_path_y[len_previous_path - 1]
						- previous_path_y[len_previous_path - 2],
				previous_path_x[len_previous_path - 1]
						- previous_path_x[len_previous_path - 2]);
		origin_x = previous_path_x[len_previous_path - 1];
		origin_y = previous_path_y[len_previous_path - 1];
	}

	bool must_reduce_speed = false;
	double cur_min_speed_mph = speed_limit_mph;
	double cur_min_distance = large_distance;
	bool must_change_lanes = false;

	check_lane_change_finished();

	Vehicle v;
	for (auto const& vehicle_map_entry : vehicles) {
		v = vehicle_map_entry.second;

		double vehicle_speed_in_mph = getNorm(v.vx, v.vy) / MPH2METPS;

		if (!is_in_current_lane(v.d))
			continue;
		if (distance(v.x, v.y, car_x, car_y) < MIN_DISTANCE_TO_OTHERS
				&& distance(car_x, car_y, v.x, v.y) < cur_min_distance
				&& vehicle_is_in_front(v) && vehicle_is_slower(v))
		{
			cur_min_distance = distance(car_x, car_y, v.x, v.y);
			must_reduce_speed = true;
			cur_min_speed_mph = std::min(cur_min_speed_mph,
					vehicle_speed_in_mph);
			if (previous_lane == -1 && cur_min_speed_mph < 45) {
				must_change_lanes = true;
				not_yet_changed_lanes = true;
			}
			cout << "reducing speed because of vehicle id " << v.id << " to "
					<< vehicle_speed_in_mph << endl;
		}
	}

	if (!must_change_lanes && previous_lane == -1
			&& current_desired_lane != 1) {
		must_change_lanes = true;
		not_yet_changed_lanes = true;
	}

	if (must_change_lanes && not_yet_changed_lanes) {
		if (current_desired_lane == 0 && previous_lane != 1) {
			if (check_lane_safe(1)) {
				trigger_lane_change(1);
				must_change_lanes = false;
			}
		} else {
			// lane is 1 or 2
			if (previous_lane != (current_desired_lane - 1)
					&& check_lane_safe(current_desired_lane - 1)) {
				trigger_lane_change(current_desired_lane - 1);
				must_change_lanes = false;

			} else if (previous_lane != (current_desired_lane + 1)
					&& current_desired_lane == 1
					&& check_lane_safe(current_desired_lane + 1)) {
				trigger_lane_change(current_desired_lane + 1);
				must_change_lanes = false;
			}
		}
		if (must_change_lanes) {
			cout << "lane change is not safe... will try next iteration." << endl;
		}
	}

	if (must_reduce_speed) {
		double new_target_speed = std::max(target_speed - 1, cur_min_speed_mph);
		cout << "reducing speed to " << new_target_speed << endl;
		target_speed = new_target_speed;
	} else {
		double new_target_speed = speed_limit_mph;
		cout << "increasing speed to " << new_target_speed << endl;
		target_speed = new_target_speed;
	}

	double optimal_d = get_center_of_lane(current_desired_lane);

	double cur_s = getFrenet(origin_x, origin_y, end_of_prev_path_yaw_rate,
			landmarks)[0];

	vector<double> future_values;
	if (not_yet_changed_lanes) {
		// for smooth change of lanes
		future_values = {50, 70, 90};
	}
	else {
		future_values = {13, 20, 35, 60};
	}

	spline_X.push_back(
			getXY(cur_s + future_values[0], optimal_d, this->landmarks)[0]);
	spline_Y.push_back(
			getXY(cur_s + future_values[0], optimal_d, this->landmarks)[1]);

	spline_X.push_back(
			getXY(cur_s + future_values[1], optimal_d, this->landmarks)[0]);
	spline_Y.push_back(
			getXY(cur_s + future_values[1], optimal_d, this->landmarks)[1]);

	spline_X.push_back(
			getXY(cur_s + future_values[2], optimal_d, this->landmarks)[0]);
	spline_Y.push_back(
			getXY(cur_s + future_values[2], optimal_d, this->landmarks)[1]);

	transform_origin_for_vector(spline_X, spline_Y, origin_x, origin_y,
			end_of_prev_path_yaw_rate);

	// spline lib needs sorted values
	sort_values(spline_X, spline_Y);

	for (int i = 0; i < len_previous_path; i++) {
		next_x_vals.push_back(this->previous_path_x[i]);
		next_y_vals.push_back(this->previous_path_y[i]);
	}

	tk::spline spline;
	spline.set_points(spline_X, spline_Y); // currently it is required that X is already sorted

	double target_x = 30;
	double target_y = spline(30);
	double target_dist = sqrt((target_x) * (target_x) + (target_y) * target_y);

	double num_points_within_dist = (target_dist
			/ (UPDATE_INTERVAL * target_speed * MPH2METPS));
	// new iteration -> reset distance variables, as now the origin has moved:
	x_prev = 0;

	for (int i = 0; i < 50 - len_previous_path; ++i) {
		double max_s = 1 / num_points_within_dist * target_x;

		double next_x = get_next_x(max_s);
		assert(next_x < spline_X[spline_X.size() - 1]);
		double next_y = spline(next_x);

		transform_back(next_x, next_y, origin_x, origin_y,
				end_of_prev_path_yaw_rate);

		next_x_vals.push_back(next_x);
		next_y_vals.push_back(next_y);
	}
}
