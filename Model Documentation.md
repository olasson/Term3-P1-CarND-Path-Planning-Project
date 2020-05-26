# Path Planning Project
Self-Driving Car Engineer Nanodegree Program

by olasson

[//]: # (Images)
[image1]: ./images/fernet_reminder.png

## Introduction

In this project, the goal is to design an algorithm that generates smooth paths for the ego car (the vehicle being directly controlled by the algorithm). The setting is a simulated 3 lane highway populated with traffic. The main, overall goal is to drive a distance of (at least) 4.32 miles with the following constraints:

* The car does not exceed a given speed limit
* The car does not exceed a total acceleration of 10 m/s^2 
* The car does not exceed a total jerk of 10 m/s^3
* The car must not collide with any other vehicle 
* The car must stays in one of the three lanes, except when changing lanes. When a lane change occurs, the car does not spend more than 3 seconds "between" lanes
* The car is able to smoothly change lanes when appropriate

The algorithm is not "explicitly" defined as a state machine, although the Path Planner Module does use some internal state variables that are updated based on the current state of the car (see lines 41, 42, 43 in p`path_planner.cpp`).

## Data 

Please see README.md for an overview of the car localization data. The map data can be found in the folder "data".

## Fernet coordinates

Udacity Lesson 10: Slide 17:

<p align="center">
     <img src="./imgages/fernet_reminder.png" alt="pipeline" width="50%" height="50%">
     <br>fernet_reminder.png
</p>


## Module: Predict Obstacles

Header: `predict_obstacles.h`
Implementation: `predict_obstacles.cpp`

## Module: Path Planner 

Header: `path_planner.h`

Implementation: `path_planner.cpp`

### Submodule: Cost
Header: `cost.h`

Implementation: `cost.cpp`

## Module: Generate Trajectory
Header: `generate_trajectory.h`

Implementation: `generate_trajectory.cpp`

## Conclusion

