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

#### ReferenceTrajectoryCritic params
 | Parameter                        | Type   | Definition                                                                                                                         |
 | ---------------                  | ------ | -----------------------------------------------------------------------------------------------------------                        |
 | reference_cost_weight            | double |                                                                                                                                    |
 | reference_cost_power             | int    |                                                                                                                                    |
 | enable_nearest_goal_critic       | bool   | enable critic that scores by mean distance from generated trajectories to nearest to generated trajectories path points            |
 | nearest_goal_offset              | int    | take offseted nearest path point [nearest + offset] in considiration                                                               |
 | nearest_goal_count               | int    | take nearest path points [nearest + offset, nearest + offset + count] in considiration                                             |
 | nearset_goal_cost_weight         | int    |                                                                                                                                    |
 | enable_nearest_path_angle_critic | bool   | enable critic that scores by mean angle difference between generated trajectories and nearest to generated trajectories path point |
 | nearest_path_angle_offset        | int    | take offseted nearest path point [nearest + offset] in considiration                                                               |
 | nearest_path_angle_cost_power    | int    |                                                                                                                                    |
 | nearest_path_angle_cost_weight   | int    |                                                                                                                                    |

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
    FollowPath:
      plugin: "mppi::Controller"
      time_steps: 15
      model_dt: 0.1
      batch_size: 300
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 1.0
      vx_max: 0.5
      vy_max: 0.5
      wz_max: 1.3
      iteration_count: 1
      prune_distance: 1.2
      transform_tolerance: 0.1
      temperature: 0.25
      motion_model: "DiffDrive"
      visualize: false
      critics: ["ReferenceTrajectoryCritic", "LocalGoalCritic", "GoalCritic", "GoalAngleCritic", "ObstaclesCritic" ]
      GoalCritic:
        goal_cost_power: 1
        goal_cost_weight: 3.0
      GoalAngleCritic:
        goal_angle_cost_power: 1
        goal_angle_cost_weight: 3.0
        threshold_to_consider_goal_angle: 0.20
      ObstaclesCritic:
        consider_footprint: true
        collision_cost: 100.0
        obstacle_cost_power: 2
        obstacle_cost_weight: 1.15
      LocalGoalCritic:
        distance_goal_count: 2
        distance_offset: 10
        angle_offset: 6
        angle_cost_power: 1
        angle_cost_weight: 3.0
        distance_cost_power: 2
        distance_cost_weight: 3.0
        stop_usage_path_reached_ratio: 0.35
      ReferenceTrajectoryCritic:
        reference_cost_power: 1
        reference_cost_weight: 4.0
        trajectory_point_step: 2
        reference_point_step: 1
      # PreferForwardCritic:
      #   prefer_forward_cost_power: 1
      #   prefer_forward_cost_weight: 10.0
      # PathAngleCritic:
      #   path_angle_cost_power: 1
      #   path_angle_cost_weight: 0.5
      # TwirlingCritic:
      #   twirling_cost_power: 1
      #   twirling_cost_weight: 25.0
```

## Topics

| Topic                     | Type                             | Description                                                           |
|---------------------------|----------------------------------|-----------------------------------------------------------------------|
| `trajectories`            | `visualization_msgs/MarkerArray` | Randomly generated trajectories, including resulting control sequence |
| `transformed_global_plan` | `nav_msgs/Path`                  | Part of global plan considered by local planner                       |

## Notes to Users

The `model_dt` parameter generally should be set to the duration of your control frequency. So if your control frequency is 20hz, this should be `0.05`. However, you may also set it lower **but not larger**.

Visualization of the trajectories using `visualize` uses compute resources to back out trajectories for visualization and therefore slows compute time. It is not suggested that this parameter is set to `true` during a deployed use, but is a useful debug instrument while tuning the system.

