// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#include <catch2/catch_all.hpp>

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

#include "utils/utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;


TEST_CASE("Optimizer doesn't fail")
{
  // Settings
  int batch_size = 400;
  int time_steps = 15;
  unsigned int path_points = 50u;
  int iteration_count = 1;
  double lookahead_distance = 10.0;

  bool consider_footprint = GENERATE(true, false);
  std::string motion_model = GENERATE("DiffDrive", "Omni");
  std::string critic = GENERATE(
    as<std::string>{},
     "",
    "GoalCritic",
    "GoalAngleCritic",
    "ObstaclesCritic",
    "LocalGoalCritic",
    "PreferForwardCritic",
    "ReferenceTrajectoryCritic",
    "PathAngleCritic",
    "TwirlingCritic");

  TestCostmapSettings cost_map_settings{};
  TestPose start_pose = cost_map_settings.getCenterPose();
  TestOptimizerSettings optimizer_settings{batch_size, time_steps, iteration_count, lookahead_distance, motion_model, consider_footprint};

  double path_step = cost_map_settings.resolution;
  TestPathSettings path_settings{start_pose, path_points, path_step, path_step};

  std::vector<std::string> critics;
  if (!critic.empty()) {
    critics.push_back(std::move(critic));
  }

  print_info(optimizer_settings, path_settings, critics);
  auto costmap_ros = getDummyCostmapRos(cost_map_settings);
  auto node = getDummyNode(optimizer_settings, critics);
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

#ifdef DO_BENCHMARKS
  BENCHMARK("evalControl Benchmark") {
    return optimizer.evalControl(pose, velocity, path, dummy_goal_checker);
  };
#else
  REQUIRE_NOTHROW(optimizer.evalControl(pose, velocity, path, dummy_goal_checker));
#endif
}


TEST_CASE("Ackermann Throw Exception on predict")
{
  mppi::xtensor::AckermannMotionModel model;
  xt::xtensor<double, 2> in;
  mppi::xtensor::models::StateIdxes idx;
  REQUIRE(!model.isHolonomic());
  REQUIRE_THROWS_AS(model.predict(in, idx), std::runtime_error);
}
