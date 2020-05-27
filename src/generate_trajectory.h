#ifndef GENERATE_TRAJECTORY_H_
#define GENERATE_TRAJECTORY_H_

#include <cmath>
#include <vector>

#include "spline.h"
#include "helpers.h"
#include "obstacle.h"
#include "action.h"

using std::vector;

vector<vector<double>> generate_trajectory(
	double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, 
	double car_lane, double car_target_velocity, bool change_lane, int prev_size,
    vector<double> map_waypoints_x,
  	vector<double> map_waypoints_y,
  	vector<double> map_waypoints_s,
  	vector<double> previous_path_x,
  	vector<double> previous_path_y);


#endif // GENERATE_TRAJECTORY_H_