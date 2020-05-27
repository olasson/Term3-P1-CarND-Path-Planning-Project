#include "next_action.h"

const double VELOCITY_INCREMENT = 0.1;       
const double MAX_VELOCITY_INCREMENT = 0.19;   

const double OBSTACLE_CLOSE_AHEAD  = 50;        

const double MAX_COST = 9999; 


double update_ego_velocity(double obstacle_ahead_distance, double obstacle_ahead_velocity, double target_velocity){
    
    // No obstacle ahead, try to speed up
    if (obstacle_ahead_distance == 9999 || obstacle_ahead_distance > OBSTACLE_CLOSE_AHEAD) {
        if (target_velocity < MAX_TARGET_VELOCITY ) { 
            target_velocity += VELOCITY_INCREMENT;
        }
    }
    // Ego car is moving faster than obstacle ahead, slow down
    else if (target_velocity > obstacle_ahead_velocity){                   
        if ((target_velocity - obstacle_ahead_velocity) <= MAX_VELOCITY_INCREMENT ){
            target_velocity  = obstacle_ahead_velocity;
        } else {
            target_velocity -= MAX_VELOCITY_INCREMENT;
        }          
    }         
    return target_velocity;
}


Action next_action(const vector<Obstacle> &predicted_obstacles, double target_velocity, int lane) {

    Obstacle obstacle_ahead  = predicted_obstacles[0];
    Obstacle obstacle_behind = predicted_obstacles[1];
    Obstacle obstacle_left_ahead   = predicted_obstacles[2]; 
    Obstacle obstacle_left_behind  = predicted_obstacles[3];
    Obstacle obstacle_right_ahead  = predicted_obstacles[4];
    Obstacle obstacle_right_behind = predicted_obstacles[5];

    Action action = {lane, target_velocity, false};

    // No obstacle ahead, keep current lane and try to speed up
    if ( obstacle_ahead.relative_distance == 9999 ||  obstacle_ahead.relative_distance > OBSTACLE_CLOSE_AHEAD ) {

       if (action.target_velocity < MAX_TARGET_VELOCITY) { 
           action.target_velocity += VELOCITY_INCREMENT;
       }
        
       //return std::make_tuple(lane_, target_speed_, change_lane_); 
       return action;
    }

    // Initialize cost variables
    double cost_keep_current_lane; 
    double cost_change_to_left_lane; 
    double cost_change_to_right_lane;
    vector<double> total_costs = {};

    // Compute cost for keeping current lane
    cost_keep_current_lane = compute_cost(obstacle_ahead.relative_distance, obstacle_behind.relative_distance, obstacle_ahead.current_velocity, target_velocity);
    total_costs.push_back(cost_keep_current_lane);    
  
    // Compute cost for changing to left lane
    if ( lane > 0 ) {  
        cost_change_to_left_lane = compute_cost(obstacle_left_ahead.relative_distance, obstacle_left_behind.relative_distance, obstacle_left_ahead.current_velocity, target_velocity);      
    }
    else {
        cost_change_to_left_lane = MAX_COST;      
    } 
    total_costs.push_back(cost_change_to_left_lane);
   
    // Compute cost for changing to right lane
    if ( lane < 2 ) {
        cost_change_to_right_lane = compute_cost(obstacle_right_ahead.relative_distance, obstacle_right_behind.relative_distance, obstacle_right_ahead.current_velocity, target_velocity);      
    } else {
        cost_change_to_right_lane = MAX_COST;
    }     
    total_costs.push_back(cost_change_to_right_lane);

    // Compute the minimum cost
    vector<double>::iterator min_cost = min_element(begin(total_costs), end(total_costs));
    int min_cost_index = distance(begin(total_costs), min_cost);

    if (min_cost_index == 0) {
        action.lane = lane; // Stay in current lane
        action.target_velocity = update_ego_velocity(obstacle_ahead.relative_distance, obstacle_ahead.current_velocity, target_velocity);
        action.change_lane = false;
    }
    else if (min_cost_index == 1) { 
        action.lane = lane - 1 ; // Change to left lane
        action.target_velocity = update_ego_velocity(obstacle_left_ahead.relative_distance, obstacle_left_ahead.current_velocity, target_velocity);
        action.change_lane = true;
    }
    else {
        action.lane = lane + 1; // Change to left lane
        action.target_velocity = update_ego_velocity(obstacle_right_ahead.relative_distance, obstacle_right_ahead.current_velocity, target_velocity);
        action.change_lane = true;
    }

    //return std::make_tuple(lane_, target_speed_, change_lane_);
    return action;

} 