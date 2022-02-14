#include <geometry_msgs/msg/detail/point__struct.hpp>
#ifdef DO_BENCHMARKS
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#endif

#include <catch2/catch.hpp>

#include <rclcpp/executors.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>

#include "mppic/StateModels.hpp"
#include "mppic/impl/Optimizer.hpp"

#include "config.hpp"
#include "factory.hpp"
#include "utils.hpp"


TEST_CASE("Next Control", "[nothrow]")
{
  auto node = factory::getDummyNode(rclcpp::NodeOptions{});
  // costmap set up
  auto costmap_ros = factory::getDummyCostmapRos();
  auto costmap_ptr = costmap_ros->getCostmap();
  auto costmap = factory::getDummyCostmap(40, 40, 0, 0, 0.1, 0);
  *(costmap_ptr) = *costmap;
  costmap_ros->setRobotFootprint({ factory::getDummyPoint(1, 1),
    factory::getDummyPoint(-1, -1),
    factory::getDummyPoint(1, -1),
    factory::getDummyPoint(-1, 1) });

  auto model = factory::getDummyModel();
  auto optimizer = factory::getDummyOptimizer(node, costmap_ros, model);

  unsigned int poses_count = GENERATE(10U, 30U, 100U);

  SECTION("Running evalControl")
  {
    auto pose = factory::getDummyPointStamped(node, 5, 5);
    auto twist = factory::getDummyTwist();
    auto path = factory::getDummyPath(poses_count, node);
    CHECK_NOTHROW(optimizer.evalControl(pose, twist, path));

#ifdef DO_BENCHMARKS
    WARN("Path with " << poses_count);
    BENCHMARK("evalControl Benchmark") { return optimizer.evalControl(ps, twist, path); };
#endif
  }
}

TEST_CASE("Optimizer with costmap2d and obstacles", "[collision]")
{
  // Node Params
  std::vector<rclcpp::Parameter> params_;
  rclcpp::NodeOptions options;
  config::setUpOptimizerParams(5, 30, 5, params_);
  options.parameter_overrides(params_);

  auto node = factory::getDummyNode(options);
  auto model = factory::getDummyModel();

  // costmap set up
  auto costmap_ros = factory::getDummyCostmapRos();
  auto costmap_ptr = costmap_ros->getCostmap();
  auto costmap = factory::getDummyCostmap(40, 40, 0, 0, 0.1, 0);
  *(costmap_ptr) = *costmap;
  costmap_ros->setRobotFootprint({ factory::getDummyPoint(1, 1),
    factory::getDummyPoint(-1, -1),
    factory::getDummyPoint(1, -1),
    factory::getDummyPoint(-1, 1) });

  auto optimizer = factory::getDummyOptimizer(node, costmap_ros, model);
  utils::addObstacle(*costmap_ptr, 5, 7, 9, 255);

  SECTION("evalControl doesn't produce path that crosses the obstacles")
  {
    size_t points_count = 40;
    float start_x = GENERATE(1.0f, 0.4f);
    float start_y = 0.4f;
    float step_x = 0.015f;
    float step_y = 0.035f;
    auto velocity = factory::getDummyTwist();
    auto pose = factory::getDummyPointStamped(node, start_x, start_y);
    auto path =
      factory::getIncrementalDummyPath(start_x, start_y, step_x, step_y, points_count, node);

    CHECK_NOTHROW(optimizer.evalControl(pose, velocity, path));

    auto trajectory = optimizer.evalTrajectoryFromControlSequence(pose, velocity);

#ifdef TEST_DEBUG_INFO
    utils::printMapWithTrajectory(*costmap, trajectory);
#endif
    CHECK(!utils::inCollision(trajectory, *costmap));
  }
}
