#include "generate_trajectory.h"


int const N_PATH_POINTS = 50; // Max number of points in a trajectory

vector<vector<double>> generate_trajectory(const EgoCar &ego_car, const Action &next_action,
    vector<double> map_waypoints_x,
    vector<double> map_waypoints_y,
    vector<double> map_waypoints_s,
    vector<double> previous_path_x,
    vector<double> previous_path_y){


    // Set reference points as the car/local coordinates
    double ref_x = ego_car.x;
    double ref_y = ego_car.y;
    double ref_yaw = deg2rad(ego_car.yaw);

    // Points for creating a spline
    vector<double> ptsx;
    vector<double> ptsy;
            
    // If the previous path is close to empty, use ego car as the starting reference
    int prev_size = previous_path_x.size();
    if (prev_size < 2) { 
        double prev_car_x = ego_car.x - cos(ego_car.yaw);
        double prev_car_y = ego_car.y - sin(ego_car.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(ego_car.x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(ego_car.y);
    } 
    // Set the previous paths end points as starting reference
    else {  
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    } 

    // Additional points for creating spline
    vector<double> next_point(2);
    double dist = 30.0;
    if (next_action.change_lane) { 
    	dist = 40;
    }
    
    for (int i = 1; i <= 3; ++i) {
        next_point = getXY((ego_car.s + dist * i), (2 + 4 * next_action.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        ptsx.push_back(next_point[0]);
        ptsy.push_back(next_point[1]);
    } 

    // Convert from map (global) coordinates to car (local) coordinates  
    for (int i = 0; i < ptsx.size(); ++i) {      

        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw)); 
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw)); 
    }
    

    tk::spline ego_spline;
    ego_spline.set_points(ptsx, ptsy);

    // Moved from main to here
    vector<double> next_x_vals;
    vector<double> next_y_vals;
  
    // Add points from previous_path to future path  
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
    
   	// Compute how to "parse" spline points in order to travel at a target velocity
    double target_x = 30.0; 
    if (next_action.change_lane) { 
    	target_x = 40;
    }
    
    double target_y = ego_spline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    
    double x_start = 0;
   
    double N = (target_dist/(0.02 * next_action.target_velocity));
    double offset = (target_x) / N;

    //Fill up the rest of path, max N_PATH_POINTS number of points
    for (int i = 1; i <= N_PATH_POINTS - previous_path_x.size(); ++i) {

        double x_point = x_start + offset;
        double y_point = ego_spline(x_point);
        x_start = x_point;

        // Transform back to global coordinates
        double x_global = ref_x +  (x_point * cos(ref_yaw) - y_point*sin(ref_yaw));
        double y_global = ref_y +  (x_point * sin(ref_yaw) + y_point*cos(ref_yaw));

        next_x_vals.push_back(x_global);
        next_y_vals.push_back(y_global);

    }
  
    return {next_x_vals, next_y_vals};

}