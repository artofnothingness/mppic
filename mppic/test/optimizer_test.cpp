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

#include "mppic/optimization/OptimizerImpl.hpp"
#include "mppic/optimization/StateModels.hpp"
#include "utils/config.hpp"
#include "utils/factory.hpp"
#include "utils/utils.hpp"

TEST_CASE("Optimizer doesn't fail", "[]")
{
  // costmap
  unsigned int cells_x = 40;
  unsigned int cells_y = 40;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double resolution = 0.1;
  unsigned char cost_map_default_value = 0;

  // start pose
  double start_x = static_cast<double>(cells_x) * resolution / 2.0;
  double start_y = static_cast<double>(cells_y) * resolution / 2.0;

  auto costmap_ros = factory::getDummyCostmapRos(
    cells_x, cells_y, origin_x, origin_y, resolution, cost_map_default_value);
  costmap_ros->setRobotFootprint(factory::getDummySquareFootprint(0.01));
  auto costmap = costmap_ros->getCostmap();
  auto model = factory::getDummyModel();

  auto create_node =
    [](int iteration_count, int time_steps, double lookahead_distance, std::string motion_model) {
      std::vector<rclcpp::Parameter> params;
      rclcpp::NodeOptions options;
      config::setUpOptimizerParams(
        iteration_count, time_steps, lookahead_distance, motion_model, params);
      options.parameter_overrides(params);
      return factory::getDummyNode(options);
    };

#ifdef DO_BENCHMARKS
  SECTION("Benchmarks")
  {
    // node
    const int iteration_count = 10;
    const int time_steps = 10;
    const double lookahead_distance = 2.0;
    auto motion_model = GENERATE(as<std::string>{}, "diff", "omni");
    unsigned int poses_count = GENERATE(10U, 30U, 100U);

    auto node = create_node(iteration_count, time_steps, lookahead_distance, motion_model);
    auto optimizer = factory::getDummyOptimizer(node, costmap_ros, model);
    auto path = factory::getDummyPath(poses_count, node);
    auto pose = factory::getDummyPointStamped(node, start_x, start_y);
    auto velocity = factory::getDummyTwist();

    WARN(
      "Points in path " << poses_count << "\niteration_count " << iteration_count << "\ntime_steps "
                        << time_steps << "\nMotion model : " << motion_model);
    BENCHMARK("evalControl Benchmark") { return optimizer.evalControl(pose, velocity, path); };
  }
#endif

  SECTION("Collision avoidance works, goal reached")
  {
    // node
    const int iteration_count = 10;
    const int time_steps = 70;
    const double lookahead_distance = 5.0;
    auto motion_model = GENERATE(as<std::string>{}, "diff", "omni");

    // obstacle
    unsigned int obstacle_size = 3;
    unsigned char obstacle_cost = 255;

    // path
    unsigned int path_points_count = 7;
    double step_x = resolution;
    double step_y = resolution;

    unsigned int center_cells_x = cells_x / 2;
    unsigned int center_cells_y = cells_y / 2;
    unsigned int y_offset = 2;
    unsigned int x_offset = 4;
    utils::addObstacle(
      *costmap, center_cells_x - x_offset, center_cells_y + y_offset, obstacle_size, obstacle_cost);
    utils::addObstacle(
      *costmap, center_cells_x + x_offset, center_cells_y + y_offset, obstacle_size, obstacle_cost);
    utils::addObstacle(
      *costmap, center_cells_x + 3 * x_offset, center_cells_y + y_offset, obstacle_size,
      obstacle_cost);

    WARN(
      "Points in path " << path_points_count << "\niteration_count " << iteration_count
                        << "\ntime_steps " << time_steps << "\nMotion model : " << motion_model);

    auto node = create_node(iteration_count, time_steps, lookahead_distance, motion_model);
    auto optimizer = factory::getDummyOptimizer(node, costmap_ros, model);

    auto pose = factory::getDummyPointStamped(node, start_x, start_y);
    auto velocity = factory::getDummyTwist();
    auto path =
      factory::getIncrementalDummyPath(start_x, start_y, step_x, step_y, path_points_count, node);

    CHECK_NOTHROW(optimizer.evalControl(pose, velocity, path));
    auto trajectory = optimizer.evalTrajectoryFromControlSequence(pose, velocity);

    auto goal_point = path.poses.back();
#ifdef TEST_DEBUG_INFO
    utils::printMapWithTrajectoryAndGoal(*costmap, trajectory, goal_point);
#endif
    CHECK(!utils::inCollision(trajectory, *costmap));
    CHECK(utils::isGoalReached(trajectory, *costmap, goal_point));
  }

  costmap_ros->on_cleanup(rclcpp_lifecycle::State{});
}
