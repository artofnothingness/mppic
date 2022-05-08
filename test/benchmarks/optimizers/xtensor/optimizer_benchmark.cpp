#include <benchmark/benchmark.h>
#include <string>

// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
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

#include "mppic/optimizers/xtensor/optimizer.hpp"
#include "mppic/optimizers/xtensor/motion_models.hpp"

#include "mppic/parameters_handler.hpp"

#include "utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};

RosLockGuard g_rclcpp;

static void BM_xtensorOptimizerDiffFootprint(benchmark::State& state)
{
  bool consider_footprint = true;
  std::string motion_model = "DiffDrive";

  // Settings
  TestCostmapSettings cost_map_settings{};
  TestOptimizerSettings optimizer_settings{12, 80, 5.0, motion_model, consider_footprint};

  const double path_step = cost_map_settings.resolution;
  TestPose start_pose = cost_map_settings.getCenterPose();
  TestPathSettings path_settings{start_pose, 8, path_step, path_step};

  auto costmap_ros = getDummyCostmapRos(cost_map_settings);
  auto node = getDummyNode(optimizer_settings);
  auto parameters_handler = std::make_unique<mppi::ParametersHandler>(node);
  auto optimizer = getDummyOptimizer(node, costmap_ros, parameters_handler.get());

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

  for (auto _ : state) {
    optimizer.evalControl(pose, velocity, path, dummy_goal_checker);
  }

}

BENCHMARK(BM_xtensorOptimizerDiffFootprint);

BENCHMARK_MAIN();
