#ifndef PREDICT_OBSTACLES_H_
#define PREDICT_OBSTACLES_H_

#include "obstacle.h"

#include <vector>
#include <cmath>

using std::vector; 

vector<Obstacle> predict_obstacles(const vector<vector<double>> &sensor_fusion, double car_s, int prev_size, int lane);

#endif // PREDICT_OBSTACLES_H_