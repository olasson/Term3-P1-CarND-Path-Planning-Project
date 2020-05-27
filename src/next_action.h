#ifndef NEXT_ACTION_H_
#define NEXT_ACTION_H_


#include <vector>
#include <iterator>
#include <tuple>

#include "obstacle.h"
#include "cost.h"
#include "action.h"
//#include "ego_car.h"

using std::vector;
using std::tuple;

/*struct NextAction {
	int lane;
	double target_velocity;
	bool change_lane;
}*/

double update_ego_velocity(double obstacle_ahead_distance, double obstacle_ahead_velocity, double target_velocity);

//tuple<int, double, bool> next_action(const vector<Obstacle> &predictions, double target_speed, int lane);
Action next_action(const vector<Obstacle> &predictions, double target_speed, int lane);

#endif // NEXT_ACTION_H_