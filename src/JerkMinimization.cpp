#include <cmath>
#include <iostream>
#include <vector>

#include "Eigen/Dense"

using std::vector;
using Eigen::Matrix3d;
using Eigen::Vector3d;

vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */

	Matrix3d A;
	A << pow(T, double(3)), pow(T, double(4)), pow(T, double(5)),
			3.0 * pow(T, double(2)), 4.0 * pow(T, double(3)), 5.0 * pow(T, double(4)),
			6.0 * T, 12.0 * pow(T, double(2)), 20.0 * pow(T, double(3));
	Vector3d b;
	b << end[0] - (start[0] + start[1] * T + 1.0/2.0 * start[2] * pow(T, double(2))),
			end[1] - (start[1] + start[2] * T),
			end[2] - start[2];

	Vector3d x = A.colPivHouseholderQr().solve(b);

  return {start[0],start[1],1.0/2.0 * start[2],x(0),x(1),x(2)};
}
