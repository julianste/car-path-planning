/*
 * PathGenerator.h
 *
 *  Created on: 12 Jun 2019
 *      Author: julian
 */

#ifndef SRC_PATHGENERATOR_H_
#define SRC_PATHGENERATOR_H_

#include <vector>
#include "Vehicle.h"

using std::vector;

class PathGenerator {
public:
	struct landmark_s {
		double x;  // iteration
		double y;
		double s;
		double d_x;
		double d_y;
	};


	PathGenerator(vector<landmark_s> &landmarks);

	virtual ~PathGenerator();

	void reset();

	void updateOtherCars(vector<vector<double>> sensorData);
	void update_ego_state(vector<double>previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d,
			double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);

	double get_next_x(double desired_x);
	void get_next_vals(vector<double> &next_x_vals, vector<double> &next_y_vals);
	int get_current_lane();

	bool check_lane_safe(int lane_number);
	bool is_in_current_lane(double d);
	bool is_in_target_lane_close_to_center(double v_d);
	bool is_near_same_lane(double, int);

	vector<landmark_s> landmarks;

	// track other vehicles
	std::map<int, Vehicle> vehicles;

	// track own quantities
	vector<double> previous_path_x, previous_path_y;
	double car_x, car_y, car_s, car_d, car_yaw, car_speed;
	double end_path_s, end_path_d;
	int current_desired_lane;
	double target_speed;
	bool not_yet_changed_lanes;
	int previous_lane;
	int timer;

	double dist_prev, dist_prevprev, x_prev;

	// some constants
	static constexpr double speed_limit_ms = 22.352;
	static constexpr double speed_limit_mph = 49.5;
	static constexpr double no_moves_every_s = 50;
	static constexpr double accel_max = 5; // in m/s
	static constexpr double jerk_max = 5; // in m/s
	static constexpr double large_distance = 10000;

private:
	bool is_almost_in_lane(int lane, double d);
	void reset_timer_if_necessary();
	void check_lane_change_finished();
	bool vehicle_is_in_front(const Vehicle& v);
	bool vehicle_is_slower(const Vehicle& v);
	void trigger_lane_change(int target_lane);
	int get_center_of_lane(int lane_number);
	void sort_values(std::vector<double>& spline_X,
			std::vector<double>& spline_Y);
};




#endif /* SRC_PATHGENERATOR_H_ */
