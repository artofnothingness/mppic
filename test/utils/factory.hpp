// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "mppic/optimizers/xtensor/motion_models.hpp"
#include "mppic/optimizers/xtensor/optimizer.hpp"
#include "mppic/parameters_handler.hpp"
#include "mppic/controller.hpp"

#include "models.hpp"

namespace detail
{
auto setHeader(auto && msg, auto node, std::string frame)
{
  auto time = node->get_clock()->now();
  msg.header.frame_id = frame;
  msg.header.stamp = time;
}

}  // namespace detail
//
//
/**
 * Adds some parameters for the optimizer to a special container.
 *
 * @param params_ container for optimizer's parameters.
 */
void setUpOptimizerParams(
  const TestOptimizerSettings & s,
  const std::vector<std::string> & critics,
  std::vector<rclcpp::Parameter> & params_, std::string node_name = std::string("dummy"))
{
  constexpr double dummy_freq = 10.0;
  params_.emplace_back(rclcpp::Parameter(node_name + ".iteration_count", s.iteration_count));
  params_.emplace_back(rclcpp::Parameter(node_name + ".batch_size", s.batch_size));
  params_.emplace_back(rclcpp::Parameter(node_name + ".time_steps", s.time_steps));
  params_.emplace_back(rclcpp::Parameter(node_name + ".lookahead_dist", s.lookahead_distance));
  params_.emplace_back(rclcpp::Parameter(node_name + ".motion_model", s.motion_model));
  params_.emplace_back(rclcpp::Parameter(node_name + ".critics", critics));
  params_.emplace_back(rclcpp::Parameter("controller_frequency", dummy_freq));
}

void setUpControllerParams(
  bool visualize, std::vector<rclcpp::Parameter> & params_,
  std::string node_name = std::string("dummy"))
{
  double dummy_freq = 10.0;
  params_.emplace_back(rclcpp::Parameter(node_name + ".visualize", visualize));
  params_.emplace_back(rclcpp::Parameter("controller_frequency", dummy_freq));
}

rclcpp::NodeOptions getOptimizerOptions(
  TestOptimizerSettings s,
  const std::vector<std::string> & critics)
{
  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions options;
  setUpOptimizerParams(s, critics, params);
  options.parameter_overrides(params);
  return options;
}

geometry_msgs::msg::Point getDummyPoint(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0;

  return point;
}
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> getDummyCostmapRos()
{
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");
  costmap_ros->on_configure(rclcpp_lifecycle::State{});
  return costmap_ros;
}

std::shared_ptr<nav2_costmap_2d::Costmap2D> getDummyCostmap(TestCostmapSettings s)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    s.cells_x, s.cells_y, s.resolution, s.origin_x, s.origin_y, s.cost_map_default_value);

  return costmap;
}

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> getDummyCostmapRos(TestCostmapSettings s)
{
  auto costmap_ros = getDummyCostmapRos();
  auto costmap_ptr = costmap_ros->getCostmap();
  auto costmap = getDummyCostmap(s);
  *(costmap_ptr) = *costmap;

  return costmap_ros;
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
getDummyNode(
  TestOptimizerSettings s, std::vector<std::string> critics,
  std::string node_name = std::string("dummy"))
{
  auto node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, getOptimizerOptions(s, critics));
  return node;
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
getDummyNode(rclcpp::NodeOptions options, std::string node_name = std::string("dummy"))
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);
  return node;
}

mppi::xtensor::Optimizer getDummyOptimizer(auto node, auto costmap_ros, auto * params_handler)
{
  auto optimizer = mppi::xtensor::Optimizer();
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> weak_ptr_node{node};

  optimizer.initialize(weak_ptr_node, node->get_name(), costmap_ros, params_handler);

  return optimizer;
}

mppi::Controller getDummyController(auto node, auto tf_buffer, auto costmap_ros)
{
  auto controller = mppi::Controller();
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> weak_ptr_node{node};

  controller.configure(weak_ptr_node, node->get_name(), tf_buffer, costmap_ros);
  controller.activate();
  return controller;
}

auto getDummyTwist()
{
  geometry_msgs::msg::Twist twist;
  return twist;
}

geometry_msgs::msg::PoseStamped
getDummyPointStamped(auto & node, std::string frame = std::string("odom"))
{
  geometry_msgs::msg::PoseStamped point;
  detail::setHeader(point, node, frame);

  return point;
}

geometry_msgs::msg::PoseStamped getDummyPointStamped(auto & node, TestPose pose)
{
  geometry_msgs::msg::PoseStamped point = getDummyPointStamped(node);
  point.pose.position.x = pose.x;
  point.pose.position.y = pose.y;

  return point;
}

nav_msgs::msg::Path getDummyPath(auto node, std::string frame = std::string("odom"))
{
  nav_msgs::msg::Path path;
  detail::setHeader(path, node, frame);
  return path;
}

auto getDummyPath(size_t points_count, auto node)
{
  auto path = getDummyPath(node);

  for (size_t i = 0; i < points_count; i++) {
    path.poses.push_back(getDummyPointStamped(node));
  }

  return path;
}

nav_msgs::msg::Path getIncrementalDummyPath(auto node, TestPathSettings s)
{
  auto path = getDummyPath(node);

  for (size_t i = 0; i < s.poses_count; i++) {
    double x = s.start_pose.x + static_cast<double>(i) * s.step_x;
    double y = s.start_pose.y + static_cast<double>(i) * s.step_y;
    path.poses.push_back(getDummyPointStamped(node, TestPose{x, y}));
  }

  return path;
}

std::vector<geometry_msgs::msg::Point> getDummySquareFootprint(double a)
{
  return {getDummyPoint(a, a), getDummyPoint(-a, -a), getDummyPoint(a, -a), getDummyPoint(-a, a)};
}
