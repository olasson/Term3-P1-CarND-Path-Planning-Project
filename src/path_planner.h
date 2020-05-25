#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_


#include <vector>
#include <iterator>
#include <tuple>

#include "obstacle.h"
#include "cost.h"

using std::vector;
using std::tuple;

double update_ego_velocity(double obstacle_ahead_distance, double obstacle_ahead_velocity, double target_velocity);

tuple<int, double, bool> path_planner(const vector<Obstacle> &predictions, double target_speed, int lane);

#endif // PATH_PLANNER_H_