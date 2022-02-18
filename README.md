#MPPI Local Planner

Diff-drive
![](.resources/demo-diff.gif)

Omni (experimental, requires adding some critics)
![](.resources/demo-omni.gif)

## Overview

Navigation2 Controller plugin. Currently testing on ros2 foxy.

This is a controller (local trajectory planner) that implements model predictive 
path integral control to track a path with collision avoidance. 

The main idea of the algorithm is to sample batch of control sequences with specified time step for each control, 
Having inital state of robot (pose, velocity) and batch of controls use iteratively "model" to predict real velocities for each time step in batch.

this can be explained as follows V(t+1) = M(t), where 
  - V(t+1) - predicted velocities of batch at time step t + 1
  - M(T) - Function that predicts real velocities at t + 1 step, by given velocities and control actions at t step.

Then velocities integrated to get trajectories. For each trajectory, the cost function is calculated. 
All control sequences are weighted by trajectories costs using softmax function to get final control sequence.

## Dependencies 
MPPIc package requires a modern C++ compiler supporting C++17, and Conan C++ package manager:
```
pip install conan
```

## Configuration

 Parameter       | Type   | Definition                                                                                                   |
| ---------------                  | ------ | -----------------------------------------------------------------------------------------------------------                                                                                                                                    |
| iteration_count                  | int    | Iteration count in MPPI algorithm                                                                                                                                                                                                              |
| lookahead_dist                   | double | Max lenght of the global plan that considered by local planner                                                                                                                                                                                    |
| batch_size                       | int    | Count of randomly sampled trajectories                                                                                                                                                                                                         |
| time_steps                       | int    | Number of time steps (points) in each sampled trajectory                                                                                                                                                                                 |
| model_dt                         | double | Time interval between two sampled points in trajectories                                                                                                                                                                                       |
| vx_std                           | double | Sampling standart deviation for VX
| vy_std                           | double | Sampling standart deviation for VY
| wx_std                           | double | Sampling Standart deviation for WX
| vx_max                           | double | Max VX
| vy_max                           | double | Max VY 
| wz_max                           | double | Max WZ 
| temperature                      | double | Selectiveness of trajectories by their costs (The closer this value to 0, the "more" we take in considiration controls with less cost), 0 mean use control with best cost, huge value will lead to just taking mean of all trajectories withou cost consideration |
| approx_reference_cost            | bool   | Use approximate point to segment distance calculation                                                                                                                                                                                          |
| visualize                        | bool   | Use visualization                                                                                                                                                                                                                              |
| goal_weight                      | double |                                                                                                                                                                                                                                                |
| goal_power                       | int    |                                                                                                                                                                                                                                                |
| reference_cost_weight            | double |                                                                                                                                                                                                                                                |
| reference_cost_power             | int    |                                                                                                                                                                                                                                                |
| obstacle_cost_weight             | double |                                                                                                                                                                                                                                                |
| obstacle_cost_power              | int    |                                                                                                                                                                                                                                                |
| goal_angle_cost_weight           | double |                                                                                                                                                                                                                                                |
| goal_angle_cost_power            | int    |                                                                                                                                                                                                                                                |
| inflation_cost_scaling_factor    | int    | Must be set accurately according to inflation layer params                                                                                                                                                                                     |
| inflation_radius                 | double | Must be set accurately according to inflation layer params                                                                                                                                                                                     |
| threshold_to_consider_goal_angle | double | Minimal distance between robot and goal above which angle goal cost considered                                                                                                                                                                 |

Example fully-described XML with default parameter values:

```
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 1.0
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    FollowPath:
      plugin: "mppi::Controller<float>"
      time_steps: 15
      model_dt: 0.1
      batch_size: 300
      vx_std: 0.1
      vy_std: 0.1
      wz_std: 0.40
      vx_max: 0.5
      vy_max: 0.5
      wz_max: 1.3
      iteration_count: 2
      temperature: 0.25
      approx_reference_cost: false
      motion_model: "diff"
      visualize: true
      CriticScorer:
        reference_cost_power: 1
        reference_cost_weight: 5
        goal_cost_power: 1
        goal_cost_weight: 15
        goal_angle_cost_power: 1
        goal_angle_cost_weight: 15
        obstacle_cost_power: 1
        obstacle_cost_weight: 1
        inflation_cost_scaling_factor: 3.0
        inflation_radius: 0.75
        threshold_to_consider_goal_angle: 0.20
```

## Topics

| Topic                     | Type                             | Description                                                           |
|---------------------------|----------------------------------|-----------------------------------------------------------------------|
| `trajectories`            | `visualization_msgs/MarkerArray` | Randomly generated trajectories, including resulting control sequence |
| `transformed_global_plan` | `nav_msgs/Path`                  | Part of global plan considered by local planner                       |

## References
[AutoRally](https://github.com/AutoRally/autorally)
