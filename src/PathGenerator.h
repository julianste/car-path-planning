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

	void updateOtherCars(vector<vector<double>> sensorData);
	void updateState(vector<double>previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d,
			double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);

	void get_next_vals(vector<double> &next_x_vals, vector<double> &next_y_vals);
	int get_current_lane();

	vector<landmark_s> landmarks;

	// track other vehicles
	std::map<int, Vehicle> vehicles;

	// track own quantities
	vector<double> previous_path_x, previous_path_y;
	double car_x, car_y, car_s, car_d, car_yaw, car_speed, end_path_s, end_path_d;
	int current_lane;
};




#endif /* SRC_PATHGENERATOR_H_ */
