# MPPI Local Planner 

## Overview

Implementation of model predictive path integral control algorithm mostly based on this paper: 
https://ieeexplore.ieee.org/document/7487277


This is a controller (local trajectory planner) that implements model predictive 
path integral control to track a path with collision avoidance. 

This plugin implements the nav2_core::Controller interface allowing it to be used across the navigation stack as a local trajectory planner in the controller server's action server (controller_server).

This controller has been measured to run at well over 10 Hz on a modern intel processor 
with path lenght = ~40, batch = 300, and time_step = 20.

## Configuration

 Parameter       | Type   | Default | Definition                                                                                                  |
| ---------------                  | ------ | ------- | ----------------------------------------------------------------------------------------------------------- |
| iteration_count                  | int    | 2       | Iteration count in MPPI algorithm                                                                           |
| lookahead_dist                   | double | 1.0     | Maximum global plan distance avaliable to local planner                                                     |
| batch_size                       | int    | 300     | Count of randmomly sampled trajectories                                                                     |
| time_steps                       | int    | 20      | Number of points propagated in time in each sampled trajectory                                              |
| model_dt                         | double | 0.1     | Time interval between two points in sampled trajectories                                                    |
| v_std                            | double | 0.1     | Standart deviation for linear speed sampling                                                                |
| w_std                            | double | 0.3     | Standart deviation for angular speed sampling
| v_limit                          | double | 0.5     | Linear speed control limit                                                                                  |
| w_limit                          | double | 1.3     | Angular speed control limit                                                                                 |
| temperature                      | double | 0.25    | Parameter set selectiveness of trajectories by their costs                                                  |
| goal_weight                      | double | 20      |                                                                                                             |
| goal_power                       | int    | 1       |                                                                                                             |
| reference_cost_weight            | double | 5       |                                                                                                             |
| reference_cost_power             | int    | 1       |                                                                                                             |
| obstacle_cost_weight             | double | 10     |                                                                                                             |
| obstacle_cost_power              | int    | 1       |                                                                                                             |
| goal_angle_cost_weight           | double | 5       |                                                                                                             |
| goal_angle_cost_power            | double | 1       |                                                                                                             |
| inflation_cost_scaling_factor    | int    | 3       | Must be set accurately according to inflation layer params                                                  |
| inflation_radius                 | double | 0.75    | Must be set accurately according to inflation layer params                                                  |
| threshold_to_consider_goal_angle | double | 0.25    | Minimal distance between robot and goal above which angle goal cost considered                              |
| approx_reference_cost            | bool   | False   | Use approximate point to segment distance calculation                                                       |
| visualize                        | bool   | True    | Use visualization                                                                                           |


## Topics

| Topic                     | Type                                   | Description                                                 |
|---------------------------|----------------------------------------|-------------------------------------------------------------|
| `trajectories`            | `visualization_msgs::msg::MarkerArray` | Randomly generated trajectories, including resulting control sequence |
| `transformed_global_plan` | `nav_msgs/Path`                        | Part of global plan considered by local planner             |
