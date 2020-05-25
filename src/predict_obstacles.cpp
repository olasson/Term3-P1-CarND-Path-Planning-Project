#include "predict_obstacles.h"

using std::vector;

double const MAX_DISTANCE_INIT = 9999; // Arbitrary large number
double const MIN_DISTANCE_INIT = -1; // Arbitrary small number
double const VELOCITY_INIT = 0; 


vector<Obstacle> predict_obstacles(const vector<vector<double>> &sensor_fusion, double car_s, int prev_size, int lane){


    double min_s = MAX_DISTANCE_INIT;
    double max_s = MIN_DISTANCE_INIT; 

    double min_left_s  = MAX_DISTANCE_INIT;
    double max_left_s  = MIN_DISTANCE_INIT;

    double min_right_s = MAX_DISTANCE_INIT;   
    double max_right_s = MIN_DISTANCE_INIT;


    Obstacle obstacle_ahead = {MAX_DISTANCE_INIT, VELOCITY_INIT};
    Obstacle obstacle_behind = {MAX_DISTANCE_INIT, VELOCITY_INIT};
    Obstacle obstacle_left_ahead   = {MAX_DISTANCE_INIT, VELOCITY_INIT};
    Obstacle obstacle_left_behind  = {MAX_DISTANCE_INIT, VELOCITY_INIT};
    Obstacle obstacle_right_ahead  = {MAX_DISTANCE_INIT, VELOCITY_INIT};
    Obstacle obstacle_right_behind = {MAX_DISTANCE_INIT, VELOCITY_INIT};
   
    // Loop through each obstacle
    for(int i = 0; i < sensor_fusion.size(); i++){
        
        // Get position and velocity of the obstacle being checked
        double velocity_x = sensor_fusion[i][3];
        double velocity_y = sensor_fusion[i][4];
        double obstacle_velocity = sqrt((velocity_x * velocity_x) + (velocity_y * velocity_y));
        double current_obstacle_s = sensor_fusion[i][5];
        float current_obstacle_d = sensor_fusion[i][6];

        // Predict the future s of current obstacle
        current_obstacle_s += ((double) prev_size * 0.02 * obstacle_velocity);
        
        double relative_distance = current_obstacle_s - car_s;

        
        // Same lane check
        if (current_obstacle_d > (4 * lane) && current_obstacle_d < ((4 * lane) + 4)) {              
            // Ahead
            if (relative_distance > 0 &&  current_obstacle_s < min_s){
                obstacle_ahead.relative_distance  = relative_distance;
                obstacle_ahead.current_velocity = obstacle_velocity;
                min_s = current_obstacle_s;
            } 
            // Behind
            else if (relative_distance < 0 && current_obstacle_s > max_s) { 
                obstacle_behind.relative_distance  = abs(relative_distance);
                obstacle_behind.current_velocity = obstacle_velocity;
                max_s = current_obstacle_s;
            }   
        } 
        // Left lane check
        else if (lane > 0 && current_obstacle_d > ((4 * lane) - 4) && current_obstacle_d < (4 * lane)) {
            // Ahead
            if (relative_distance > 0 &&  current_obstacle_s < min_left_s){
                obstacle_left_ahead.relative_distance  = relative_distance;
                obstacle_left_ahead.current_velocity = obstacle_velocity;
                min_left_s = current_obstacle_s;
            } 
            // Behind
            else if (relative_distance < 0 && current_obstacle_s > max_left_s) { 
                obstacle_left_behind.relative_distance  = abs(relative_distance);
                obstacle_left_behind.current_velocity = obstacle_velocity;
                max_left_s = current_obstacle_s;
            }   
        } 
        // Right lane check
        else {
            // Ahead
            if (relative_distance > 0 &&  current_obstacle_s < min_right_s) {
                obstacle_right_ahead.relative_distance  = relative_distance;
                obstacle_right_ahead.current_velocity = obstacle_velocity;
                min_right_s = current_obstacle_s;
            } 
            // Behind
            else if (relative_distance < 0 && current_obstacle_s > max_right_s) { 
                obstacle_right_behind.relative_distance  = abs(relative_distance);
                obstacle_right_behind.current_velocity = obstacle_velocity;
                max_right_s = current_obstacle_s;
            }   
        } 
         
    }

    // NOTE: the order of push_back is not arbitrary. 
    vector<Obstacle> predicted_obstacles;
    predicted_obstacles.push_back(obstacle_ahead);
    predicted_obstacles.push_back(obstacle_behind);
    predicted_obstacles.push_back(obstacle_left_ahead);
    predicted_obstacles.push_back(obstacle_left_behind);
    predicted_obstacles.push_back(obstacle_right_ahead);
    predicted_obstacles.push_back(obstacle_right_behind);


    return predicted_obstacles;

}