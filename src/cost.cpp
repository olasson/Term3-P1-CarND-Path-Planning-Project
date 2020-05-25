#include "cost.h"

// Weights for total cost
const double W_DISTANCE = 50; 
const double W_VELOCITY = 40;

const double BUFFER_DISTANCE = 35;

double compute_distance_cost(double distance){
    double cost;
    
    if (distance < BUFFER_DISTANCE) { 
        cost = 35 - distance;
    } 
    else {
        cost = (1.0 / distance);
    }
    return cost;

} 

double compute_cost(double obstacle_distance_ahead, double obstacle_distance_behind, 
    double obstacle_velocity_ahead, double ego_velocity){

    double cost_distance = std::max(compute_distance_cost(obstacle_distance_ahead) , compute_distance_cost(obstacle_distance_behind));

    double cost_velocity;

    if ( obstacle_velocity_ahead >= ego_velocity ) {
        cost_velocity = (MAX_TARGET_VELOCITY - ego_velocity) / MAX_TARGET_VELOCITY;
    } else  {
        cost_velocity = ego_velocity - obstacle_velocity_ahead ;
    } 

    double total_cost = (W_DISTANCE * cost_distance) + (W_VELOCITY * cost_velocity);

    return total_cost;

} 