#ifdef DO_BENCHMARKS
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#endif

#include <catch2/catch.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>

#include "mppic/optimizer.hpp"
#include "mppic/motion_models.hpp"

#include "utils/utils.hpp"

#ifdef DO_BENCHMARKS
TEST_CASE("Optimizer Benchmarks", "[]")
{

  SECTION("Benchmarks")
  {
    auto motion_model = GENERATE(as<std::string>{}, "DiffDrive", "Omni");
    auto poses_count = GENERATE(10U, 30U, 100U);

    TestOptimizerSettings optimizer_settings{10, 10, 2.0, motion_model};
    TestCostmapSettings cost_map_settings{};

    auto costmap_ros = getDummyCostmapRos(cost_map_settings);
    auto costmap = costmap_ros->getCostmap();
    costmap_ros->setRobotFootprint(getDummySquareFootprint(0.01));

    TestPose start_pose = cost_map_settings.getCenterPose();

    print_info(optimizer_settings, poses_count);

    auto node = getDummyNode(optimizer_settings);
    auto optimizer = getDummyOptimizer(node, costmap_ros, getDummyModel());

    auto path = getDummyPath(poses_count, node);
    auto pose = getDummyPointStamped(node, start_pose);
    auto velocity = getDummyTwist();

    BENCHMARK("evalControl Benchmark") {
      return optimizer.evalControl(pose, velocity, path);
    };
  }
}
#endif

TEST_CASE("Optimizer doesn't fail", "[]")
{
  SECTION("Collision avoidance works, goal reached")
  {
    auto consider_footprint = GENERATE(true, false);
    auto motion_model = GENERATE(as<std::string>{}, "DiffDrive", "Omni");

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

    CHECK_NOTHROW(optimizer.evalControl(pose, velocity, path));
    auto trajectory = optimizer.evalTrajectoryFromControlSequence(pose, velocity);
    auto goal_point = path.poses.back();
#ifdef TEST_DEBUG_INFO
    printMapWithTrajectoryAndGoal(*costmap, trajectory, goal_point);
#endif
    CHECK(!inCollision(trajectory, *costmap));
    CHECK(isGoalReached(trajectory, *costmap, goal_point));
  }
}
