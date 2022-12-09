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
#include "mppic/optimizer.hpp"

// Tests main optimizer functions

// applyControlSequenceConstraints - Check min/max respected. Ackermann rad too.
// updateInitialStateVelocities - Check first values are initials
// propagateStateVelocitiesFromInitials - check pass through on cvx/vx
// getControlFromSequenceAsTwist - set sequence and check returns valid object with right info
// integrateStateVelocities - Give it a couple of easy const traj and check rollout

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace mppi;  // NOLINT
using namespace mppi::critics;  // NOLINT
using namespace mppi::utils;  // NOLINT
using xt::evaluation_strategy::immediate;

class OptimizerTester : public Optimizer
{
public:
  OptimizerTester()
  : Optimizer() {}

  void testSetDiffModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    EXPECT_NO_THROW(setMotionModel("DiffDrive"));
    EXPECT_NE(motion_model_.get(), nullptr);
    EXPECT_TRUE(dynamic_cast<DiffDriveMotionModel *>(motion_model_.get()));
    EXPECT_FALSE(isHolonomic());
  }

  void testSetOmniModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    EXPECT_NO_THROW(setMotionModel("Omni"));
    EXPECT_NE(motion_model_.get(), nullptr);
    EXPECT_TRUE(dynamic_cast<OmniMotionModel *>(motion_model_.get()));
    EXPECT_TRUE(isHolonomic());
  }

  void testSetAckModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    EXPECT_NO_THROW(setMotionModel("Ackermann"));
    EXPECT_NE(motion_model_.get(), nullptr);
    EXPECT_TRUE(dynamic_cast<AckermannMotionModel *>(motion_model_.get()));
    EXPECT_FALSE(isHolonomic());
  }

  void testSetRandModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    try {
      setMotionModel("Random");
      FAIL();
    } catch (...) {
      SUCCEED();
    }
    EXPECT_EQ(motion_model_.get(), nullptr);
  }

  void resetMotionModel()
  {
    motion_model_.reset();
  }

  void setOffsetWrapper(const double freq)
  {
    return setOffset(freq);
  }

  bool getShiftControlSequence()
  {
    return settings_.shift_control_sequence;
  }

  void fillOptimizerWithGarbage()
  {
    state_.vx = 0.43432 * xt::ones<float>({1000, 10});
    control_sequence_.vx = 342.0 * xt::ones<float>({30});
    control_history_[0] = {43, 5646, 32432};
    costs_ = 5.32 * xt::ones<float>({56453});
    generated_trajectories_.x = 432.234 * xt::ones<float>({7865, 1});
  }

  void testReset()
  {
    reset();

    EXPECT_EQ(state_.vx, xt::zeros<float>({1000, 50}));
    EXPECT_EQ(control_sequence_.vx, xt::zeros<float>({50}));
    EXPECT_EQ(control_history_[0].vx, 0.0);
    EXPECT_EQ(control_history_[0].vy, 0.0);
    EXPECT_NEAR(xt::sum(costs_, immediate)(), 0, 1e-6);
    EXPECT_EQ(generated_trajectories_.x, xt::zeros<float>({1000, 50}));
  }

  bool fallbackWrapper(bool fail)
  {
    return fallback(fail);
  }

  void testPrepare(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
  {
    prepare(robot_pose, robot_speed, plan, goal_checker);

    EXPECT_EQ(critics_data_.goal_checker, nullptr);
    EXPECT_NEAR(xt::sum(costs_, immediate)(), 0, 1e-6);  // should be reset
    EXPECT_FALSE(critics_data_.fail_flag);  // should be reset
    EXPECT_FALSE(critics_data_.motion_model->isHolonomic());  // object is valid + diff drive
    EXPECT_EQ(state_.pose.pose.position.x, 999);
    EXPECT_EQ(state_.speed.linear.y, 4.0);
    EXPECT_EQ(path_.x.shape(0), 17u);
  }

  void testShiftControlSequence()
  {
    control_sequence_.reset({100});
    control_sequence_.vx(0) = 9999;
    control_sequence_.vx(1) = 6;
    control_sequence_.vx(2) = 888;
    control_sequence_.vy(0) = 9999;
    control_sequence_.vy(1) = 6;
    control_sequence_.vy(2) = 888;
    control_sequence_.wz(0) = 9999;
    control_sequence_.wz(1) = 6;
    control_sequence_.wz(2) = 888;

    resetMotionModel();
    testSetOmniModel();
    shiftControlSequence();

    EXPECT_EQ(control_sequence_.vx(0), 6);
    EXPECT_EQ(control_sequence_.vy(0), 6);
    EXPECT_EQ(control_sequence_.wz(0), 6);
    EXPECT_EQ(control_sequence_.vx(1), 888);
    EXPECT_EQ(control_sequence_.vy(1), 888);
    EXPECT_EQ(control_sequence_.wz(1), 888);
    EXPECT_EQ(control_sequence_.vx(2), 0);
    EXPECT_EQ(control_sequence_.vy(2), 0);
    EXPECT_EQ(control_sequence_.wz(2), 0);
  }

  void testSpeedLimit()
  {
    auto & s = settings_;
    EXPECT_EQ(s.constraints.vx_max, 0.5);
    EXPECT_EQ(s.constraints.vx_min, -0.35);
    setSpeedLimit(0, false);
    EXPECT_EQ(s.constraints.vx_max, 0.5);
    EXPECT_EQ(s.constraints.vx_min, -0.35);
    setSpeedLimit(50.0, true);
    EXPECT_NEAR(s.constraints.vx_max, 0.5 / 2.0, 1e-3);
    EXPECT_NEAR(s.constraints.vx_min, -0.35 / 2.0, 1e-3);
    setSpeedLimit(0, true);
    EXPECT_EQ(s.constraints.vx_max, 0.5);
    EXPECT_EQ(s.constraints.vx_min, -0.35);
    setSpeedLimit(0.75, false);
    EXPECT_NEAR(s.constraints.vx_max, 0.75, 1e-3);
    EXPECT_NEAR(s.constraints.vx_min, -0.5249, 1e-2);
  }
};

TEST(OptimizerTests, BasicInitializedFunctions)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Should be empty of size batches x time steps
  // and tests getting set params: time_steps, batch_size, controller_frequency
  auto trajs = optimizer_tester.getGeneratedTrajectories();
  EXPECT_EQ(trajs.x.shape(0), 1000u);
  EXPECT_EQ(trajs.x.shape(1), 50u);
  EXPECT_EQ(trajs.x, xt::zeros<float>({1000, 50}));

  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  auto traj = optimizer_tester.getOptimizedTrajectory();
  EXPECT_EQ(traj(5, 0), 0.0);  // x
  EXPECT_EQ(traj(5, 1), 0.0);  // y
  EXPECT_EQ(traj(5, 2), 0.0);  // yaw
  EXPECT_EQ(traj.shape(0), 50u);
  EXPECT_EQ(traj.shape(1), 3u);

  optimizer_tester.shutdown();
}

TEST(OptimizerTests, TestOptimizerMotionModels)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Diff Drive should be non-holonomic
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetDiffModel();

  // Omni Drive should be holonomic
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  // // Ackermann should be non-holonomic
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetAckModel();

  // // Rand should fail
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetRandModel();
}

TEST(OptimizerTests, setOffsetTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.1));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test offsets are properly set based on relationship of model_dt and controller frequency
  // Also tests getting set model_dt parameter.
  EXPECT_THROW(optimizer_tester.setOffsetWrapper(1.0), std::runtime_error);
  EXPECT_NO_THROW(optimizer_tester.setOffsetWrapper(30.0));
  EXPECT_FALSE(optimizer_tester.getShiftControlSequence());
  EXPECT_NO_THROW(optimizer_tester.setOffsetWrapper(10.0));
  EXPECT_TRUE(optimizer_tester.getShiftControlSequence());
}

TEST(OptimizerTests, resetTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Tests resetting the full state of all the functions after filling with garbage
  optimizer_tester.fillOptimizerWithGarbage();
  optimizer_tester.testReset();
}

TEST(OptimizerTests, FallbackTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test fallback logic, also tests getting set param retry_attempt_limit
  // Because retry set to 2, it should attempt soft resets 2x before throwing exception
  // for hard reset
  EXPECT_FALSE(optimizer_tester.fallbackWrapper(false));
  EXPECT_TRUE(optimizer_tester.fallbackWrapper(true));
  EXPECT_TRUE(optimizer_tester.fallbackWrapper(true));
  EXPECT_THROW(optimizer_tester.fallbackWrapper(true), std::runtime_error);
}

TEST(OptimizerTests, PrepareTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test Prepare function to set the state of the robot pose/speed on new cycle
  // Populate the contents with things easily identifiable if correct
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 999;
  geometry_msgs::msg::Twist speed;
  speed.linear.y = 4.0;
  nav_msgs::msg::Path path;
  path.poses.resize(17);

  optimizer_tester.testPrepare(pose, speed, path, nullptr);
}

TEST(OptimizerTests, shiftControlSequenceTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test shiftControlSequence by setting the 2nd value to something unique to neighbors
  optimizer_tester.testShiftControlSequence();
}

TEST(OptimizerTests, SpeedLimitTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test Speed limits API
  optimizer_tester.testSpeedLimit();
}
