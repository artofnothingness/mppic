#ifdef DO_BENCHMARKS
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#endif

#include "mppic/impl/Optimizer.hpp"

#include <catch2/catch.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/executors.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>

#include "mppic/StateModels.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tests_utils.hpp"

using T = float;
std::string NODE_NAME = "TestNode";

namespace detail {
auto
setHeader(auto &&msg, auto node, std::string frame) {
  auto time = node->get_clock()->now();
  msg.header.frame_id = frame;
  msg.header.stamp = time;
}
}  // namespace detail

namespace config {
/**
 * Adds some parameters for the optimizer to a special container.
 *
 * @param params_ container for optimizer's parameters.
 */
void
setUpOptimizerParams(auto iter,
                     auto time_steps,
                     auto lookahead,
                     std::vector<rclcpp::Parameter> &params_) {
  params_.push_back(rclcpp::Parameter(NODE_NAME + ".iteration_count", iter));
  params_.push_back(
      rclcpp::Parameter(NODE_NAME + ".lookahead_dist", time_steps));
  params_.push_back(rclcpp::Parameter(NODE_NAME + ".time_steps", lookahead));
}
}  // namespace config

namespace factory {
auto
getDummyCostmapRos() {
  auto costmap_ros =
      std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");

  auto st = rclcpp_lifecycle::State{};
  costmap_ros->on_configure(st);
  return costmap_ros;
}

auto
getDummyCostmap(auto size_x,
                auto size_y,
                auto origin_x,
                auto origin_y,
                auto resolution,
                auto default_value) {
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
      size_x, size_y, origin_x, origin_y, resolution, default_value);

  return costmap;
}

auto
getDummyModel() {
  auto model = mppi::models::NaiveModel<T>;
  return model;
}

auto
getDummyNode(rclcpp::NodeOptions options) {
  auto node =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(NODE_NAME, options);
  return node;
}

auto
getDummyOptimizer(auto node, auto costmap_ros, auto model) {
  auto optimizer = mppi::optimization::Optimizer<T>();
  optimizer.on_configure(node.get(), NODE_NAME, costmap_ros.get(), model);
  optimizer.on_activate();

  return optimizer;
}

auto
getDummyTwist() {
  geometry_msgs::msg::Twist twist;
  return twist;
}

auto
getDummyPoint(auto node) {
  geometry_msgs::msg::PoseStamped point;
  detail::setHeader(point, node, "dummy");

  return point;
}

auto
getDummyPoint(auto node, auto x, auto y) {
  auto point = getDummyPoint(node);
  point.pose.position.x = x;
  point.pose.position.y = y;

  return point;
}

auto
getDummyPath(auto node) {
  nav_msgs::msg::Path path;
  detail::setHeader(path, node, "dummy");
  return path;
}

auto
getDummyPath(auto points_count, auto node) {
  auto path = getDummyPath(node);

  for (size_t i = 0; i < points_count; i++) {
    path.poses.push_back(getDummyPoint(node));
  }

  return path;
}

auto
getIncrementalDummyPath(auto start_x,
                        auto start_y,
                        auto step_x,
                        auto step_y,
                        auto points_count,
                        auto node) {
  auto path = getDummyPath(node);

  for (size_t i = 0; i < points_count; i++) {
    auto x = start_x + static_cast<float>(i) * step_x;
    auto y = start_y + static_cast<float>(i) * step_y;
    path.poses.push_back(getDummyPoint(node, x, y));
  }

  return path;
}

}  // namespace factory

TEST_CASE("Next Control", "[nothrow]") {
  auto node = factory::getDummyNode(rclcpp::NodeOptions{});
  auto costmap_ros = factory::getDummyCostmapRos();
  auto model = factory::getDummyModel();
  auto optimizer = factory::getDummyOptimizer(node, costmap_ros, model);

  unsigned int poses_count = GENERATE(10U, 30U, 100U);

  SECTION("Running evalControl") {
    auto pose = factory::getDummyPoint(node);
    auto twist = factory::getDummyTwist();
    auto path = factory::getDummyPath(poses_count, node);
    CHECK_NOTHROW(optimizer.evalControl(pose, twist, path));

#ifdef DO_BENCHMARKS
    WARN("Path with " << poses_count);
    BENCHMARK("evalControl Benchmark") {
      return optimizer.evalControl(ps, twist, path);
    };
#endif
  }
}

TEST_CASE("Optimizer with costmap2d and obstacles", "[collision]") {
  // Node Params
  std::vector<rclcpp::Parameter> params_;
  rclcpp::NodeOptions options;
  config::setUpOptimizerParams(5, 30, 5, params_);
  options.parameter_overrides(params_);

  auto node = factory::getDummyNode(options);
  auto model = factory::getDummyModel();
  auto costmap_ros = factory::getDummyCostmapRos();
  auto costmap = costmap_ros->getCostmap();
  *(costmap) = *factory::getDummyCostmap(40, 40, 0, 0, 0.1, 0);
  auto optimizer = factory::getDummyOptimizer(node, costmap_ros, model);
  addObstacle(*costmap, 5, 7, 9, 255);

  SECTION("evalControl doesn't produce path crossing the obstacles") {
    size_t points_count = 40;
    float start_x = GENERATE(1.0f, 0.4f);
    float start_y = 0.4f;
    float step_x = 0.015f;
    float step_y = 0.035f;
    auto velocity = factory::getDummyTwist();
    auto pose = factory::getDummyPoint(node, start_x, start_y);
    auto path = factory::getIncrementalDummyPath(start_x, start_y, step_x, step_y,
                                        points_count, node);

    CHECK_NOTHROW(
        optimizer.evalControl(pose, velocity, path));

    auto trajectory = optimizer.evalTrajectoryFromControlSequence(
        pose, velocity);

#ifdef TEST_DEBUG_INFO
    printMapWithTrajectory(*costmap, trajectory);
#endif
    CHECK(!inCollision(trajectory, *costmap));
  }
}