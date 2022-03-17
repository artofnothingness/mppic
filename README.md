# Model Predictive Path Integral Controller

Differential                  |  Omnidirectional 
:----------------------------:|:-------------------------:
![](.resources/demo-diff.gif) | ![](.resources/demo-omni.gif)

## Overview

This is a controller (local trajectory planner) that implements the [Model Predictive Path Integral (MPPI)](https://ieeexplore.ieee.org/document/7487277) algorithm to track a path with adaptive collision avoidance. It contains plugin-based critic functions to impact the behavior of the algorithm. It was created by Aleksei Budyakov and adapted for Nav2 by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/).

This plugin implements the ``nav2_core::Controller`` interface allowing it to be used across the navigation stack as a local trajectory planner in the controller server's action server (``controller_server``).

This controller is measured to run at 45 Hz on a modest Intel processor (4th gen i5). See its Configuration Guide Page for additional parameter descriptions.

## MPPI Description

The MPPI algorithm finds a control velocity for the robot using an iterative approach. Using the previous time step's best control solution and the robot's current state, a set of randomly sampled perturbations from a Gaussian distribution are applied. These noised controls are forward simulated to generate a set of trajectories within the robot's motion model.

Next, these trajectories are scored using a set of plugin-based critic functions to find the best trajectory in the batch. The output scores are used to set the best control with a soft max function.

This process is then repeated a number of times and returns a converged solution. This solution is then used as the basis of the next time step's initial control.

## Dependencies 

This uses the usual ROS tools for dependency management, so please use ``rosdep`` to install the dependencies. 

Note: If running on Ubuntu 20.04 or other OS's that `xtensor` is not released in in binary form, please manually install `xtensor` v 0.24.0 and `xtl` v 0.7.0. These are simply headers so the install process is trivially short, unfortunately the `xtensor` project isn't available in package managers in some common-place operating systems (albeit, all necessary ROS OS versions) so you may be required to do this yourself if building from source.

```
git clone git@github.com:xtensor-stack/xtensor.git -b 0.24.0
cd xtensor
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make install

git clone git@github.com:xtensor-stack/xtl.git -b 0.7.0
cd xtl
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make install
```

## Configuration

### Controller params
 | Parameter             | Type   | Definition                                                                                                                                                                                                                                                        |
 | --------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 | motion_model | string | Type of model [DiffDrive, Omni, Ackermann]                                                                                                                                                                                                                                                                 |            
 | critics      | string | Critics (plugins) names                                                                                                                                                                                                                                                    |
 | iteration_count       | int    | Iteration count in MPPI algorithm                                                                                                                                                                                                                                 |
 | max_robot_pose_search_dist | double | Upper bound on integrated distance along the global plan to search for the closest pose to the robot pose. This should be left as the default unless there are paths with loops and intersections that do not leave the local costmap, in which case making this value smaller is necessary to prevent shortcutting. |
 | transform_tolerance   | double | TF tolerance to transform poses                                                                                                                                                                                                                                   |
 | batch_size            | int    | Count of randomly sampled trajectories                                                                                                                                                                                                                            |
 | time_steps            | int    | Number of time steps (points) in each sampled trajectory                                                                                                                                                                                                          |
 | model_dt              | double | Time interval between two sampled points in trajectories                                                                                                                                                                                                          |
 | vx_std                | double | Sampling standart deviation for VX                                                                                                                                                                                                                                |
 | vy_std                | double | Sampling standart deviation for VY                                                                                                                                                                                                                                |
 | wx_std                | double | Sampling standart deviation for WX                                                                                                                                                                                                                                |
 | vx_max                | double | Max VX                                                                                                                                                                                                                                                            |
 | vy_max                | double | Max VY                                                                                                                                                                                                                                                            |
 | wz_max                | double | Max WZ                                                                                                                                                                                                                                                            |
 | temperature           | double | Selectiveness of trajectories by their costs (The closer this value to 0, the "more" we take in considiration controls with less cost), 0 mean use control with best cost, huge value will lead to just taking mean of all trajectories without cost consideration |
 | visualize             | bool   | Use visualization                                                                                                                                                                                                                                                 |                                                    |

#### GoalCritic params
 | Parameter            | Type   | Definition                                                                                                  |
 | -------------------- | ------ | ----------------------------------------------------------------------------------------------------------- |
 | goal_goal_cost_weight| double |                                                                                                             |
 | goal_cost_power      | int    |                                                                                                             |

#### GoalAngleCritic params
 | Parameter                        | Type   | Definition                                                                                                  |
 | ---------------                  | ------ | ----------------------------------------------------------------------------------------------------------- |
 | goal_angle_cost_weight           | double |                                                                                                             |
 | goal_angle_cost_power            | int    |                                                                                                             |
 | threshold_to_consider_goal_angle | double | Minimal distance between robot and goal above which angle goal cost considered                              |

#### PathAngleCritic params
 | Parameter                 | Type   | Definition                                                                                                  |
 | ---------------           | ------ | ----------------------------------------------------------------------------------------------------------- |
 | path_angle_cost_weight    | double |                                                                                                             |
 | path_angle_cost_power     | int    |                                                                                                             |

#### ApproxReferenceTrajectoryCritic and ReferenceTrajectoryCritic params
 | Parameter             | Type   | Definition                                                                                                  |
 | ---------------       | ------ | ----------------------------------------------------------------------------------------------------------- |
 | reference_cost_weight | double |                                                                                                             |
 | reference_cost_power  | int    |                                                                                                             |

#### ObstaclesCritic params
 | Parameter                     | Type   | Definition                                                                                                  |
 | ---------------               | ------ | ----------------------------------------------------------------------------------------------------------- |
 | consider_footprint            | bool   |                                                                                                             |
 | obstacle_cost_weight          | double |                                                                                                             |
 | obstacle_cost_power           | int    |                                                                                                             |


### XML configuration example
```
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "mppi::Controller"
      time_steps: 15
      model_dt: 0.1
      batch_size: 400
      vx_std: 0.1
      vy_std: 0.1
      wz_std: 0.6
      vx_max: 0.5
      vy_max: 0.5
      wz_max: 1.3
      iteration_count: 2
      temperature: 0.25
      motion_model: "DiffDrive"
      visualize: false
      critics: [ "GoalCritic", "GoalAngleCritic", "PathAngleCritic", "ReferenceTrajectoryCritic", "ObstaclesCritic" ]
      GoalCritic:
        goal_cost_power: 1
        goal_cost_weight: 8.0
      GoalAngleCritic:
        goal_angle_cost_power: 1
        goal_angle_cost_weight: 15.0
        threshold_to_consider_goal_angle: 0.20
      ReferenceTrajectoryCritic:
        reference_cost_power: 1
        reference_cost_weight: 5.0
      ObstaclesCritic:
        consider_footprint: true
        obstacle_cost_power: 1
        obstacle_cost_weight: 0.5
      PathAngleCritic:
        path_angle_cost_power: 1
        path_angle_cost_weight: 0.5
```

## Topics

| Topic                     | Type                             | Description                                                           |
|---------------------------|----------------------------------|-----------------------------------------------------------------------|
| `trajectories`            | `visualization_msgs/MarkerArray` | Randomly generated trajectories, including resulting control sequence |
| `transformed_global_plan` | `nav_msgs/Path`                  | Part of global plan considered by local planner                       |

## Notes to Users

The `model_dt` parameter generally should be set to the duration of your control frequency. So if your control frequency is 20hz, this should be `0.05`. However, you may also set it lower **but not larger**.

Visualization of the trajectories using `visualize` uses compute resources to back out trajectories for visualization and therefore slows compute time. It is not suggested that this parameter is set to `true` during a deployed use, but is a useful debug instrument while tuning the system.

