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

using std::string;
using std::vector;
using std::cout;
using std::endl;


PathGenerator::PathGenerator(vector<landmark_s> &landmarks) : landmarks(landmarks) {
	this->landmarks = landmarks;
}
PathGenerator::~PathGenerator() {}

void PathGenerator::updateOtherCars(vector<vector<double>> sensor_fusion_data) {
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
    		cout << "Creating new vehicle with id " << id << " and x,y,vx,vy,s,d = " << x << "," << y << "," << vx << "," << vy
    				<< "," << s << "," << d << endl;
    		Vehicle newVehicle(id, x, y, vx,
    				vy, s, d);
    		this->vehicles[id] = newVehicle;
    	}
    	else {

    		Vehicle veh = this->vehicles.find(id)->second;
    		veh.updateVehicle(x, y, vx,
    				vy, s, d);
    	}
    }
}

void PathGenerator::updateState(vector<double>previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d,
					double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed) {
	this->previous_path_x = previous_path_x;
	this->previous_path_y = previous_path_y;
	this->end_path_s = end_path_s;
	this->end_path_d = end_path_d;

	this->car_x = car_x;
	this->car_y = car_y;
	this->car_s = car_s;
	this->car_d = car_d;
	this->car_yaw = car_yaw;
	this->car_speed = car_speed;
	this->current_lane = this->get_current_lane();
}

/**
 * gets current lane
 * 0 for leftmost, 1 middle, 2 for rightmost
 */
int PathGenerator::get_current_lane() {
	double d = this->car_d;
	return int(d / 4.0);
}


void PathGenerator::get_next_vals(vector<double> &next_x_vals, vector<double> &next_y_vals) {

	cout << "getting next values. previous_path_x.size is " << this->previous_path_x.size() << endl;

	int current_lane = this->get_current_lane();
	double optimal_d = 4 + 2 * current_lane;
    double dist_inc = 0.5;

    int n_interpolation_points = 10;
	// smooth path between multiple points. current_position is start, next landmark is end.
    // spacing in between for n_interpolation_points
	int desired_index = this->previous_path_x.size() - 25;
	double reference_x, reference_y;
	if (this->previous_path_x.size() < desired_index) {
		reference_x = this->car_x - 10; //??? wrong
		reference_y = this->car_y;
	}
	else {
		reference_x = this->previous_path_x[desired_index];
		reference_y = this->previous_path_y[desired_index];
	}
	cout << "getting frenet for old vector" << endl;
	vector<double> reference_frenet = getFrenet(reference_x, reference_y, this->car_yaw, this->landmarks);

	int next_waypoint_idx = getNextLandmark(this->car_x, this->car_y, this->car_yaw,
	this->landmarks);
	double next_waypoint_x = this->landmarks[next_waypoint_idx].x;
	double next_waypoint_y = this->landmarks[next_waypoint_idx].y;

	if (distance(next_waypoint_x, next_waypoint_y, this->car_x, this->car_y) < 10) {
		// too near -> take next
		next_waypoint_x = this->landmarks[next_waypoint_idx + 1].x;
		next_waypoint_y = this->landmarks[next_waypoint_idx + 1].y;
	}
	cout << "getting frenet for next landmark..." << endl;
	vector<double> next_waypoint_frenet = getFrenet(next_waypoint_x, next_waypoint_y, this->car_yaw, this->landmarks);


	std::vector<double> X(3), Y(3);

	X[0]=reference_frenet[0]; X[1]=this->car_s; X[2]=next_waypoint_frenet[0];
	Y[0]=reference_frenet[1]; Y[1]=this->car_d; Y[2]=optimal_d;

	tk::spline spline;
	spline.set_points(X,Y);    // currently it is required that X is already sorted

	for (int i = 0; i < 50; ++i) {
		vector<double> values = getXY(spline(this->car_s + (i+1) * dist_inc), optimal_d, this->landmarks);
		next_x_vals.push_back(values[0]);
		next_y_vals.push_back(values[1]);
	}
}
