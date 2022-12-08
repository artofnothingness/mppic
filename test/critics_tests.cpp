// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "mppic/tools/utils.hpp"
#include "mppic/motion_models.hpp"
#include "mppic/critics/constraint_critic.hpp"
#include "mppic/critics/goal_angle_critic.hpp"
#include "mppic/critics/goal_critic.hpp"
#include "mppic/critics/obstacles_critic.hpp"
#include "mppic/critics/path_align_critic.hpp"
#include "mppic/critics/path_angle_critic.hpp"
#include "mppic/critics/path_follow_critic.hpp"
#include "mppic/critics/prefer_forward_critic.hpp"
#include "mppic/critics/twirling_critic.hpp"
#include "utils_test.cpp"  // NOLINT

// Tests the various critic plugin functions

// ROS lock used from utils_test.cpp

using namespace mppi;  // NOLINT
using namespace mppi::critics;  // NOLINT
using namespace mppi::utils;  // NOLINT
using xt::evaluation_strategy::immediate;

TEST(CriticTests, ConstraintsCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();

  // Initialization testing

  // Make sure initializes correctly and that defaults are reasonable
  ConstraintCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");
  EXPECT_TRUE(critic.getMaxVelConstraint() > 0.0);
  EXPECT_TRUE(critic.getMinVelConstraint() < 0.0);

  // Scoring testing

  // provide velocities in constraints, should not have any costs
  state.vx = 0.40 * xt::ones<float>({1000, 30});
  state.vy = xt::zeros<float>({1000, 30});
  state.wz = xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);

  // provide out of maximum velocity constraint
  auto last_batch_traj_in_full = xt::view(state.vx, -1, xt::all());
  last_batch_traj_in_full = 0.60 * xt::ones<float>({30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);
  // 4.0 weight * 0.1 model_dt * 0.1 error introduced * 30 timesteps = 1.2
  EXPECT_NEAR(costs(999), 1.2, 0.01);
  costs = xt::zeros<float>({1000});

  // provide out of minimum velocity constraint
  auto first_batch_traj_in_full = xt::view(state.vx, 1, xt::all());
  first_batch_traj_in_full = -0.45 * xt::ones<float>({30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);
  // 4.0 weight * 0.1 model_dt * 0.1 error introduced * 30 timesteps = 1.2
  EXPECT_NEAR(costs(1), 1.2, 0.01);
  costs = xt::zeros<float>({1000});

  // Now with ackermann, all in constraint so no costs to score
  state.vx = 0.40 * xt::ones<float>({1000, 30});
  state.wz = 1.5 * xt::ones<float>({1000, 30});
  data.motion_model = std::make_shared<AckermannMotionModel>(&param_handler);
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);

  // Now violating the ackermann constraints
  state.wz = 2.5 * xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);
  // 4.0 weight * 0.1 model_dt * (0.2 - 0.4/2.5) * 30 timesteps = 0.48
  EXPECT_NEAR(costs(1), 0.48, 0.01);
}

TEST(CriticTests, GoalAngleCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();

  // Initialization testing

  // Make sure initializes correctly
  GoalAngleCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path too far from `threshold_to_consider` to consider
  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 10.0;
  path.y(9) = 0.0;
  path.yaws(9) = 3.14;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);

  // Lets move it even closer, just to be sure it still doesn't trigger
  state.pose.pose.position.x = 9.2;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);

  // provide state pose and path below `threshold_to_consider` to consider
  state.pose.pose.position.x = 9.7;
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);
  EXPECT_NEAR(costs(0), 9.42, 0.02);  // (3.14 - 0.0) * 3.0 weight
}

TEST(CriticTests, GoalCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();

  // Initialization testing

  // Make sure initializes correctly
  GoalCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing with all trajectories set to 0

  // provide state poses and path far
  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 10.0;
  path.y(9) = 0.0;
  critic.score(data);
  EXPECT_NEAR(costs(2), 50.0, 1e-6);  // (sqrt(10.0 * 10.0) * 5.0 weight
  EXPECT_NEAR(xt::sum(costs, immediate)(), 50000.0, 1e-6);  // Should all be 50 * 1000
  costs = xt::zeros<float>({1000});

  // provide state pose and path close
  path.x(9) = 0.5;
  path.y(9) = 0.0;
  critic.score(data);
  EXPECT_NEAR(costs(2), 2.5, 1e-6);  // (sqrt(10.0 * 10.0) * 5.0 weight
  EXPECT_NEAR(xt::sum(costs, immediate)(), 2500.0, 1e-6);  // should be 2.5 * 1000
}

TEST(CriticTests, PathAngleCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally

  // Initialization testing

  // Make sure initializes correctly
  PathAngleCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path close, within pose tolerance so won't do anything
  state.pose.pose.position.x = 0.0;
  state.pose.pose.position.y = 0.0;
  path.reset(10);
  path.x(9) = 0.15;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path close but outside of tol. with less than PI/2 angular diff.
  path.x(9) = 0.95;
  data.furthest_reached_path_point = 2;  // So it grabs the 2 + offset_from_furthest_ = 6th point
  path.x(6) = 1.0;  // angle between path point and pose = 0 < max_angle_to_furthest_
  path.y(6) = 0.0;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path close but outside of tol. with more than PI/2 angular diff.
  path.x(6) = -1.0;  // angle between path point and pose > max_angle_to_furthest_
  path.y(6) = 4.0;
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0.0);
  EXPECT_NEAR(costs(0), 3.6315, 1e-2);  // atan2(4,-1) [1.81] * 2.0 weight
}

TEST(CriticTests, PreferForwardCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally

  // Initialization testing

  // Make sure initializes correctly
  PreferForwardCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path far away, not within positional tolerances
  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 10.0;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path close to trigger behavior but with all forward motion
  path.x(9) = 0.15;
  state.vx = xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path close to trigger behavior but with all reverse motion
  state.vx = -1.0 * xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0.0);
  EXPECT_NEAR(costs(0), 15.0, 1e-6);  // 1.0 * 0.1 model_dt * 5.0 weight * 30 length
}

TEST(CriticTests, TwirlingCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally
  data.goal_checker = &goal_checker;

  // Initialization testing

  // Make sure initializes correctly
  TwirlingCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path far away, not within positional tolerances
  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 10.0;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path close to trigger behavior but with no angular variation
  path.x(9) = 0.15;
  state.wz = xt::zeros<float>({1000, 30});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // Provide nearby with some motion
  auto traj_view = xt::view(state.wz, 0, xt::all());
  traj_view = 10.0;
  critic.score(data);
  EXPECT_NEAR(costs(0), 100.0, 1e-6);  // (mean(10.0) * 10.0 weight
  costs = xt::zeros<float>({1000});

  // Now try again with some wiggling noise
  traj_view = xt::random::randn<float>({30}, 0.0, 0.5);
  critic.score(data);
  EXPECT_NEAR(costs(0), 3.3, 4e-1);  // (mean of noise with mu=0, sigma=0.5 * 10.0 weight
}
