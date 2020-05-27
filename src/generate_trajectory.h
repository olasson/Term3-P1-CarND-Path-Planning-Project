#ifndef GENERATE_TRAJECTORY_H_
#define GENERATE_TRAJECTORY_H_

#include <cmath>
#include <vector>

#include "spline.h"
#include "helpers.h"
#include "obstacle.h"
#include "action.h"
#include "ego_car.h"

using std::vector;

vector<vector<double>> generate_trajectory(const EgoCar &ego_car, const Action &next_action,
    vector<double> map_waypoints_x,
  	vector<double> map_waypoints_y,
  	vector<double> map_waypoints_s,
  	vector<double> previous_path_x,
  	vector<double> previous_path_y);


#endif // GENERATE_TRAJECTORY_H_