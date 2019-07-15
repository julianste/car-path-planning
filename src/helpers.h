#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

#include "PathGenerator.h"

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::endl;
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
inline string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() {
	return M_PI;
}
inline double deg2rad(double x) {
	return x * pi() / 180;
}
inline double rad2deg(double x) {
	return x * 180 / pi();
}

inline double getNorm(double x, double y) {
	return sqrt(x * x + y * y);
}

// Calculate distance between two points
inline double distance(double x1, double y1, double x2, double y2) {
	return getNorm(x2 - x1, y2 - y1);
}

// Calculate closest waypoint to current x, y position
inline int getClosestLandmark(double x, double y,
		const vector<PathGenerator::landmark_s> landmarks) {
	int no_of_landmarks = landmarks.size();
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < no_of_landmarks; ++i) {
		double map_x = landmarks[i].x;
		double map_y = landmarks[i].y;
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

// Returns landmark the car will encounter next (so could be closest or that after closest)
inline int getNextLandmark(double x, double y, double theta,
		const vector<PathGenerator::landmark_s> landmarks) {

	int no_of_landmarks = landmarks.size();
	int closestLandmark = getClosestLandmark(x, y, landmarks);

	double map_x = landmarks[closestLandmark].x;
	double map_y = landmarks[closestLandmark].y;

	double heading_to_landmark = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading_to_landmark);
	angle = std::min(2 * pi() - angle, angle);

	// Question: does this also work for very tight 180 degree curves?
	// Here the next landmark could be more than 90 degrees to the current heading, but it is not behind...
	bool is_landmark_behind = angle > pi() / 2;

	if (is_landmark_behind) {
		closestLandmark = (closestLandmark + 1) % no_of_landmarks;
	}

	return closestLandmark;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
inline vector<double> getFrenet(double x, double y, double theta,
		const vector<PathGenerator::landmark_s> landmarks) {

	int next_wp = getNextLandmark(x, y, theta, landmarks);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0) {
		prev_wp = landmarks.size() - 1;
	}

	double landmark_delta_x = landmarks[next_wp].x - landmarks[prev_wp].x;
	double landmark_delta_y = landmarks[next_wp].y - landmarks[prev_wp].y;
	double prev_landmark_to_x = x - landmarks[prev_wp].x;
	double prev_landmark_to_y = y - landmarks[prev_wp].y;

	// find the projection of (x,y) onto landmark_delta
	double scalar_prod = (prev_landmark_to_x * landmark_delta_x
			+ prev_landmark_to_y * landmark_delta_y);
	double norm = (landmark_delta_x * landmark_delta_x
			+ landmark_delta_y * landmark_delta_y);

	double proj_norm = scalar_prod / norm;

	// the projection of x onto the line between the two landmarks
	double proj_x = proj_norm * landmark_delta_x;
	double proj_y = proj_norm * landmark_delta_y;

	double frenet_d = distance(prev_landmark_to_x, prev_landmark_to_y, proj_x,
			proj_y);

	//see if d value is positive or negative by comparing it to a center point
	double prev_landmark_to_center_x = 1000 - landmarks[prev_wp].x;
	double prev_landmark_to_center_y = 2000 - landmarks[prev_wp].y;
	double centerToPos = distance(prev_landmark_to_center_x,
			prev_landmark_to_center_y, prev_landmark_to_x, prev_landmark_to_y);
	double centerToRef = distance(prev_landmark_to_center_x,
			prev_landmark_to_center_y, proj_x, proj_y);

	// I don't get this
	if (centerToPos < centerToRef) {
		cout << "d is negative, frenet_d * -1 = " << frenet_d * (-1) << endl;
		frenet_d *= -1;
	}

	// calculate s value = sum of distances between all waypoints...
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; ++i) {
		frenet_s += distance(landmarks[i].x, landmarks[i].y, landmarks[i + 1].x,
				landmarks[i + 1].y);
	}
	// plus last slice
	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
inline vector<double> getXY(double s, double d,
		const vector<PathGenerator::landmark_s> &landmarks) {
	int prev_wp = -1;

	int no_landmarks = landmarks.size();

	while (s > landmarks[prev_wp + 1].s && (prev_wp < (int) (no_landmarks - 1))) {
		++prev_wp;
	}

	int wp2 = (prev_wp + 1) % no_landmarks;

	double heading = atan2((landmarks[wp2].y - landmarks[prev_wp].y),
			(landmarks[wp2].x - landmarks[prev_wp].x));
	// the x,y,s along the segment
	double seg_s = (s - landmarks[prev_wp].s);

	double seg_x = landmarks[prev_wp].x + seg_s * cos(heading);
	double seg_y = landmarks[prev_wp].y + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x,y};
}

inline void transform_origin_for_vector(vector<double> &X, vector<double> &Y,
		double orig_x, double orig_y, double orig_yaw) {
	// coordinate transformation

	for (size_t i = 0; i < X.size(); i++) {
		// shift car reference to (0,0) 0 degrees
		double shifted_x = X[i] - orig_x;
		double shifted_y = Y[i] - orig_y;

		X[i] = shifted_x * cos(orig_yaw) + shifted_y * sin(orig_yaw);
		Y[i] = -shifted_x * sin(orig_yaw) + shifted_y * cos(orig_yaw);

	}
}

inline void transform_back(double &x, double &y, double orig_x,
		double orig_y, double orig_yaw) {

	// rotate back
	double tmp = x * cos(orig_yaw) - y * sin(orig_yaw);
	y = x * sin(orig_yaw) + y * cos(orig_yaw);
	x = tmp;

	// shift back
	x = x + orig_x;
	y = y + orig_y;

}

#endif  // HELPERS_H
