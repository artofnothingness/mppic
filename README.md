# Model Predictive Path Integral Controller

![](media/demo.gif)

## Overview

This is a controller (local trajectory planner) that implements the [Model Predictive Path Integral (MPPI)](https://ieeexplore.ieee.org/document/7487277) algorithm to track a path with adaptive collision avoidance. It contains plugin-based critic functions to impact the behavior of the algorithm. It was created by [Aleksei Budyakov](https://www.linkedin.com/in/aleksei-budyakov-334889224/) and adapted for Nav2 by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/).

This plugin implements the ``nav2_core::Controller`` interface allowing it to be used across the navigation stack as a local trajectory planner in the controller server's action server (``controller_server``).

This controller is measured to run at 45 Hz on a modest Intel processor (4th gen i5). See its Configuration Guide Page for additional parameter descriptions.

It works currently with Differential and Omnidirectional robots, with support for Ackermann planned.

## MPPI Description

The MPPI algorithm finds a control velocity for the robot using an iterative approach. Using the previous time step's best control solution and the robot's current state, a set of randomly sampled perturbations from a Gaussian distribution are applied. These noised controls are forward simulated to generate a set of trajectories within the robot's motion model.

Next, these trajectories are scored using a set of plugin-based critic functions to find the best trajectory in the batch. The output scores are used to set the best control with a soft max function.

This process is then repeated a number of times and returns a converged solution. This solution is then used as the basis of the next time step's initial control.

## Dependencies 

This uses the usual ROS tools for dependency management, so please use ``rosdep`` to install the dependencies. 

Note: If running on Ubuntu 20.04 or other OS's that `xtensor` is not released in binary form, please manually install `xtensor` v 0.24.0 and `xtl` v 0.7.0. These are simply headers so the install process is trivially short, unfortunately the `xtensor` project isn't available in package managers in some common-place operating systems (albeit, all necessary ROS OS versions) so you may be required to do this yourself if building from source.

```
git clone git@github.com:xtensor-stack/xtensor.git -b 0.24.0
cd xtensor
mkdir build
cd build
cmake ..
sudo make install

git clone git@github.com:xtensor-stack/xtl.git -b 0.7.0
cd xtl
mkdir build
cd build
cmake ..
sudo make install

# Optional
git clone git@github.com:xtensor-stack/xsimd.git -b 8.0.5
cd xsimd
mkdir build
cmake ..
sudo make install
```

## Configuration

### Controller params
 | Parameter                  | Type   | Definition                                                                                                                                                                                                                                                                                                           |
 | ---------------------      | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------                                                   |
 | motion_model               | string | Type of model [DiffDrive, Omni, Ackermann]                                                                                                                                                                                                                                                                           |
 | critics                    | string | Critics (plugins) names                                                                                                                                                                                                                                                                                              |
 | iteration_count            | int    | Iteration count in MPPI algorithm                                                                                                                                                                                                                                                                                    |
 | max_robot_pose_search_dist | double | Upper bound on integrated distance along the global plan to search for the closest pose to the robot pose. This should be left as the default unless there are paths with loops and intersections that do not leave the local costmap, in which case making this value smaller is necessary to prevent shortcutting. |
 | transform_tolerance        | double | TF tolerance to transform poses                                                                                                                                                                                                                                                                                      |
 | batch_size                 | int    | Count of randomly sampled trajectories                                                                                                                                                                                                                                                                               |
 | time_steps                 | int    | Number of time steps (points) in each sampled trajectory                                                                                                                                                                                                                                                             |
 | model_dt                   | double | Time interval between two sampled points in trajectories                                                                                                                                                                                                                                                             |
 | vx_std                     | double | Sampling standart deviation for VX                                                                                                                                                                                                                                                                                   |
 | vy_std                     | double | Sampling standart deviation for VY                                                                                                                                                                                                                                                                                   |
 | wx_std                     | double | Sampling standart deviation for WX                                                                                                                                                                                                                                                                                   |
 | vx_max                     | double | Max VX                                                                                                                                                                                                                                                                                                               |
 | vy_max                     | double | Max VY                                                                                                                                                                                                                                                                                                               |
 | vx_min                     | double | Min VX                                                                                                                                                                                                                                                                                                               |
 | wz_max                     | double | Max WZ                                                                                                                                                                                                                                                                                                               |
 | temperature                | double | Selectiveness of trajectories by their costs (The closer this value to 0, the "more" we take in considiration controls with less cost), 0 mean use control with best cost, huge value will lead to just taking mean of all trajectories without cost consideration                                                   |
 | gamma                | double |  A trade-off between smoothness (high) and low energy (low). This is a complex parameter that likely won't need to be changed from the default of `0.1` which works well for a broad range of cases. See Section 3D-2 in "Information Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving" for detailed information.       |
 | visualize                  | bool   | Use visualization                                                                                                                                                                                                                                                                                                    |
 | retry_attempt_limit        | int    | Number of attempts to find feasible trajectory before failure                                                                                                                                                                                                                                                      |


#### AckermannConstrains params
 | Parameter            | Type   | Definition                                                                                                  |
 | -------------------- | ------ | ----------------------------------------------------------------------------------------------------------- |
 | min_turning_r        | double | minimum turning radius for ackermann motion model                                                           |


#### GoalCritic params
 | Parameter            | Type   | Definition                                                                                                  |
 | -------------------- | ------ | ----------------------------------------------------------------------------------------------------------- |
 | goal_cost_weight     | double |                                                                                                             |
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

#### PathAlignCritic params
 | Parameter                  | Type   | Definition                                                                                                              |
 | ---------------            | ------ | -----------------------------------------------------------------------------------------------------------             |
 | path_align_cost_weight     | double |                                                                                                                         |
 | path_align_cost_power      | int    |                                                                                                                         |
 | enable_nearest_goal_critic | bool   | enable critic that scores by mean distance from generated trajectories to nearest to generated trajectories path points |
 | path_point_step            | int    | Consider path points with given step                                                                                    |
 | trajectory_point_step      | int    | Consider generated trajectories points with given step                                                                  |


#### ObstaclesCritic params
 | Parameter            | Type   | Definition                                                                                                  |
 | ---------------      | ------ | ----------------------------------------------------------------------------------------------------------- |
 | consider_footprint   | bool   |                                                                                                             |
 | obstacle_cost_weight | double |                                                                                                             |
 | obstacle_cost_power  | int    |                                                                                                             |

#### PreferForwardCritic params
 | Parameter             | Type   | Definition                                                                                                  |
 | ---------------       | ------ | ----------------------------------------------------------------------------------------------------------- |
 | prefer_forward_cost_weight | double |                                                                                                             |
 | prefer_forward_cost_power  | int    |                                                                                                             |

#### TwirlingCritic params
 | Parameter             | Type   | Definition                                                                                                  |
 | ---------------       | ------ | ----------------------------------------------------------------------------------------------------------- |
 | twirling_cost_weight | double |                                                                                                             |
 | twirling_cost_power  | int    |                                                                                                             |

### XML configuration example
```
controller_server:
  ros__parameters:
    controller_frequency: 30.0
    FollowPath:
      plugin: "mppi::Controller"
      time_steps: 30
      model_dt: 0.1
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.35
      vy_max: 0.5
      wz_max: 1.3
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.35
      motion_model: "DiffDrive"
      visualize: false
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      AckermannConstrains:
        min_turning_r: 0.2
      critics: ["ObstaclesCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider_goal_angle: 0.35
      ObstaclesCritic:
        enabled: true
        cost_power: 2
        cost_weight: 1.65
        consider_footprint: false
        collision_cost: 2000.0
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        path_point_step: 2
        trajectory_point_step: 3
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        offset_from_furthest: 10
        max_path_ratio: 0.40
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
      # TwirlingCritic:
      #   twirling_cost_power: 1
      #   twirling_cost_weight: 10.0
```
## Topics

| Topic                     | Type                             | Description                                                           |
|---------------------------|----------------------------------|-----------------------------------------------------------------------|
| `trajectories`            | `visualization_msgs/MarkerArray` | Randomly generated trajectories, including resulting control sequence |
| `transformed_global_plan` | `nav_msgs/Path`                  | Part of global plan considered by local planner                       |

## Notes to Users

The `model_dt` parameter generally should be set to the duration of your control frequency. So if your control frequency is 20hz, this should be `0.05`. However, you may also set it lower **but not larger**.

Visualization of the trajectories using `visualize` uses compute resources to back out trajectories for visualization and therefore slows compute time. It is not suggested that this parameter is set to `true` during a deployed use, but is a useful debug instrument while tuning the system.

