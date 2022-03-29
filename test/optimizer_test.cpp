// Copyright 2022 FastSense, Samsung Research
#include "gtest/gtest.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_core/goal_checker.hpp>

#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>

#include "mppic/optimizer.hpp"
#include "mppic/motion_models.hpp"

#include "utils/utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;


class ThreeColumnsAndIncrementalPath : public ::testing::Test
{
protected:
     virtual void SetUp()
     {      
      goal_checker_ = nullptr;
      start_pose_ = costmap_settings_.getCenterPose();
      path_step_ = costmap_settings_.resolution;
      path_settings_ = TestPathSettings{start_pose_, 8, path_step_, path_step_};

      // setup costmap
      costmap_ros_ = getDummyCostmapRos(costmap_settings_);
      costmap_ = costmap_ros_->getCostmap();
      {
        costmap_ros_->setRobotFootprint(getDummySquareFootprint(0.01));
        const unsigned int obst_center_x = costmap_settings_.cells_x / 2;
        const unsigned int obst_center_y = costmap_settings_.cells_y / 2 + 1;
        TestObstaclesSettings obs_settings_1{obst_center_x - 4, obst_center_y, 4, 255};
        TestObstaclesSettings obs_settings_2{obst_center_x + 4, obst_center_y, 4, 255};
        TestObstaclesSettings obs_settings_3{obst_center_x + 12, obst_center_y, 4, 255};
        addObstacle(costmap_, obs_settings_1);
        addObstacle(costmap_, obs_settings_2);
        addObstacle(costmap_, obs_settings_3);
      }

     }

     virtual void TearDown()
     {
     }

  std::string motion_model_;
  double path_step_;
  nav2_core::GoalChecker * goal_checker_;
  TestPose start_pose_;
  TestPathSettings path_settings_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  TestCostmapSettings costmap_settings_;
};

TEST_F(ThreeColumnsAndIncrementalPath, DiffDriveFootprintModelNoCollisionAndGoalReached)
{
  bool consider_footprint = true;
  std::string motion_model = "DiffDrive";

  TestOptimizerSettings optimizer_settings{12, 80, 5.0, motion_model, consider_footprint};

  auto node = getDummyNode(optimizer_settings);
  auto optimizer = getDummyOptimizer(node, costmap_ros_);

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose_);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings_);

  print_info(optimizer_settings, path_settings_);

  EXPECT_NO_THROW(optimizer.evalControl(pose, velocity, path, dummy_goal_checker));
  auto trajectory = optimizer.evalTrajectoryFromControlSequence(pose, velocity);
  auto goal_point = path.poses.back();
#ifdef TEST_DEBUG_INFO
  printMapWithTrajectoryAndGoal(*costmap_, trajectory, goal_point);
#endif
  EXPECT_TRUE(!inCollision(trajectory, *costmap_));
  EXPECT_TRUE(isGoalReached(trajectory, *costmap_, goal_point));
}

TEST(MPPIOptimizer, OptimizerTestDiffFootprint)
{
  bool consider_footprint = true;
  std::string motion_model = "DiffDrive";

  // Settings
  TestCostmapSettings cost_map_settings{};
  TestOptimizerSettings optimizer_settings{12, 80, 5.0, motion_model, consider_footprint};

  const double path_step = cost_map_settings.resolution;
  TestPose start_pose = cost_map_settings.getCenterPose();
  TestPathSettings path_settings{start_pose, 8, path_step, path_step};

  print_info(optimizer_settings, path_settings);

  auto costmap_ros = getDummyCostmapRos(cost_map_settings);
  auto node = getDummyNode(optimizer_settings);
  auto optimizer = getDummyOptimizer(node, costmap_ros);

  // setup costmap
  auto costmap = costmap_ros->getCostmap();
  {
    costmap_ros->setRobotFootprint(getDummySquareFootprint(0.01));
    const unsigned int obst_center_x = cost_map_settings.cells_x / 2;
    const unsigned int obst_center_y = cost_map_settings.cells_y / 2 + 1;
    TestObstaclesSettings obs_settings_1{obst_center_x - 4, obst_center_y, 4, 255};
    TestObstaclesSettings obs_settings_2{obst_center_x + 4, obst_center_y, 4, 255};
    TestObstaclesSettings obs_settings_3{obst_center_x + 12, obst_center_y, 4, 255};
    addObstacle(costmap, obs_settings_1);
    addObstacle(costmap, obs_settings_2);
    addObstacle(costmap, obs_settings_3);
  }

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);

  nav2_core::GoalChecker * dummy_goal_checker{nullptr};
  EXPECT_NO_THROW(optimizer.evalControl(pose, velocity, path, dummy_goal_checker));
  auto trajectory = optimizer.evalTrajectoryFromControlSequence(pose, velocity);
  auto goal_point = path.poses.back();
#ifdef TEST_DEBUG_INFO
  printMapWithTrajectoryAndGoal(*costmap, trajectory, goal_point);
#endif
  EXPECT_TRUE(!inCollision(trajectory, *costmap));
  EXPECT_TRUE(isGoalReached(trajectory, *costmap, goal_point));
}

TEST(MPPIOptimizer, OptimizerTestOmniCircle)
{
  bool consider_footprint = false;
  std::string motion_model = "Omni";

  // Settings
  TestCostmapSettings cost_map_settings{};
  TestOptimizerSettings optimizer_settings{12, 80, 5.0, motion_model, consider_footprint};

  const double path_step = cost_map_settings.resolution;
  TestPose start_pose = cost_map_settings.getCenterPose();
  TestPathSettings path_settings{start_pose, 8, path_step, path_step};

  print_info(optimizer_settings, path_settings);

  auto costmap_ros = getDummyCostmapRos(cost_map_settings);
  auto node = getDummyNode(optimizer_settings);
  auto optimizer = getDummyOptimizer(node, costmap_ros);

  // setup costmap
  auto costmap = costmap_ros->getCostmap();
  {
    costmap_ros->setRobotFootprint(getDummySquareFootprint(0.01));
    const unsigned int obst_center_x = cost_map_settings.cells_x / 2;
    const unsigned int obst_center_y = cost_map_settings.cells_y / 2 + 1;
    TestObstaclesSettings obs_settings_1{obst_center_x - 4, obst_center_y, 4, 255};
    TestObstaclesSettings obs_settings_2{obst_center_x + 4, obst_center_y, 4, 255};
    TestObstaclesSettings obs_settings_3{obst_center_x + 12, obst_center_y, 4, 255};
    addObstacle(costmap, obs_settings_1);
    addObstacle(costmap, obs_settings_2);
    addObstacle(costmap, obs_settings_3);
  }

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);

  nav2_core::GoalChecker * dummy_goal_checker{nullptr};
  EXPECT_NO_THROW(optimizer.evalControl(pose, velocity, path, dummy_goal_checker));
  auto trajectory = optimizer.evalTrajectoryFromControlSequence(pose, velocity);
  auto goal_point = path.poses.back();
#ifdef TEST_DEBUG_INFO
  printMapWithTrajectoryAndGoal(*costmap, trajectory, goal_point);
#endif
  EXPECT_TRUE(!inCollision(trajectory, *costmap));
  EXPECT_TRUE(isGoalReached(trajectory, *costmap, goal_point));
}

TEST(MPPIOptimizer, AckermannException)
{
  mppi::AckermannMotionModel model;
  xt::xtensor<double, 2> in;
  mppi::models::StateIdxes idx;
  EXPECT_FALSE(model.isHolonomic());
  EXPECT_THROW(model.predict(in, idx), std::runtime_error);
}
