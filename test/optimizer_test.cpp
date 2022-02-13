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
setUpOptimizerParams(std::vector<rclcpp::Parameter> &params_) {
  params_.push_back(rclcpp::Parameter(NODE_NAME + ".iteration_count", 5));
  params_.push_back(rclcpp::Parameter(NODE_NAME + ".lookahead_dist", 5));
  params_.push_back(rclcpp::Parameter(NODE_NAME + ".time_steps", 30));
}
}  // namespace config

namespace factory {
auto
getDummyCostmap() {
  auto costmap_ros =
      std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");

  auto st = rclcpp_lifecycle::State{};
  costmap_ros->on_configure(st);
  return costmap_ros;
}

auto
getDummyModel() {
  auto model = mppi::models::NaiveModel<T>;
  return model;
}

auto
getDummyNode() {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(NODE_NAME);
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
getDummyPath(auto points_count, auto node) {
  nav_msgs::msg::Path path;
  auto point = getDummyPoint(node);

  for (size_t i = 0; i < points_count; i++) {
    path.poses.push_back(point);
  }

  return path;
}
}  // namespace factory

TEST_CASE("Next Control", "[nothrow]") {
  auto node = factory::getDummyNode();
  auto costmap_ros = factory::getDummyCostmap();
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
  std::string node_name = "TestNode";
  auto state = rclcpp_lifecycle::State{};
  std::vector<rclcpp::Parameter> params_;
  rclcpp::NodeOptions options;
  config::setUpOptimizerParams(params_);
  options.parameter_overrides(params_);

  // args for costmap2d
  unsigned int cells_size_x = 20;
  unsigned int cells_size_y = 20;
  double resolution = 0.1;
  double origin_x = 0.0;
  double origin_y = 0.0;
  unsigned char default_value = 0;

  // args for obstacle on costmap2d
  const unsigned int upper_left_corner_x = 5;
  const unsigned int upper_left_corner_y = 7;
  const unsigned int obstacle_side_size_cells = 9;
  unsigned char obstacle_cost = 255;

  // create parameters for reference path generation
  size_t poses_count = 40;
  float x_step = 0.015f;
  float y_step = 0.035f;

  auto node =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);
  auto costmap_ros =
      std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");

  auto &model = mppi::models::NaiveModel<T>;
  auto optimizer = mppi::optimization::Optimizer<T>();

  costmap_ros->on_configure(state);
  *costmap_ros->getCostmap() = *std::make_shared<nav2_costmap_2d::Costmap2D>(
      cells_size_x, cells_size_y, resolution, origin_x, origin_y,
      default_value);

  // creating a square obstacle
  addObstacle(*costmap_ros->getCostmap(), upper_left_corner_x,
              upper_left_corner_y, obstacle_side_size_cells, obstacle_cost);

  optimizer.on_configure(node.get(), node_name, costmap_ros.get(), model);
  optimizer.on_activate();

  float start_point_x = GENERATE(1.0f, 0.4f);
  float start_point_y = 0.4f;

  SECTION("evalControl doesn't produce path crossing the obstacles") {
    std::string frame = "odom";  // frame for header in path and points
    auto time = node->get_clock()->now();  // time for header in path

    nav_msgs::msg::Path reference_path;
    geometry_msgs::msg::PoseStamped reference_goal_pose;
    geometry_msgs::msg::PoseStamped init_robot_pose;
    init_robot_pose.pose.position.x = start_point_x;
    init_robot_pose.pose.position.y = start_point_y;
    geometry_msgs::msg::Twist init_robot_vel;

    // lambda expression for setting header
    auto setHeader = [&](auto &&msg) {
      msg.header.frame_id = frame;
      msg.header.stamp = time;
    };

    // lambda expression for refernce path generation
    auto fillRealPath = [&](size_t count) {
      for (size_t i = 0; i < count; i++) {
        reference_goal_pose.pose.position.x =
            static_cast<float>(i) * x_step + start_point_x;
        reference_goal_pose.pose.position.y =
            static_cast<float>(i) * y_step + start_point_y;
        reference_path.poses.push_back(reference_goal_pose);
      }
    };

    setHeader(reference_goal_pose);
    setHeader(init_robot_pose);
    setHeader(reference_path);

    fillRealPath(poses_count);

    // update controal sequence in optimizer
    CHECK_NOTHROW(
        optimizer.evalControl(init_robot_pose, init_robot_vel, reference_path));
    // get best trajectory from optimizer
    auto trajectory = optimizer.evalTrajectoryFromControlSequence(
        init_robot_pose, init_robot_vel);

#ifdef TEST_DEBUG_INFO
    printMapWithGoalAndTrajectory(*costmap_ros->getCostmap(), trajectory,
                                  reference_goal_pose);
#endif
    CHECK(!inCollision(trajectory, *costmap_ros->getCostmap()));
  }

  optimizer.on_deactivate();
  optimizer.on_cleanup();
  costmap_ros->on_cleanup(state);
  costmap_ros.reset();
}
