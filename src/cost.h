#ifndef COST_H_
#define COST_H_

#include <algorithm>
#include <cmath>


const double SPEED_LIMIT = 22.3; 
const double BUFFER_VELOCITY = 0.2;     


const double MAX_TARGET_VELOCITY = SPEED_LIMIT - BUFFER_VELOCITY;


double compute_cost(double dist_ahead, double dist_behind, double ahead_speed, double my_speed);

double compute_distance_cost(double distance);

#endif // COST_H_