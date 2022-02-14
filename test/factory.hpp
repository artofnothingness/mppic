#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace factory {

using T = float;
namespace detail {
  auto setHeader(auto &&msg, auto node, std::string frame)
  {
    auto time = node->get_clock()->now();
    msg.header.frame_id = frame;
    msg.header.stamp = time;
  }
}// namespace detail


geometry_msgs::msg::Point getDummyPoint(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0;

  return point;
}
auto getDummyCostmapRos()
{
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");

  auto st = rclcpp_lifecycle::State{};
  costmap_ros->on_configure(st);
  return costmap_ros;
}

auto getDummyCostmap(auto size_x,
  auto size_y,
  auto origin_x,
  auto origin_y,
  auto resolution,
  auto default_value)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    size_x, size_y, origin_x, origin_y, resolution, default_value);

  return costmap;
}

auto getDummyModel()
{
  auto model = mppi::models::NaiveModel<T>;
  return model;
}

auto getDummyNode(rclcpp::NodeOptions options, std::string node_name = std::string("dummy"))
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);
  return node;
}

auto getDummyOptimizer(auto node, auto costmap_ros, auto model)
{
  auto optimizer = mppi::optimization::Optimizer<T>();
  optimizer.on_configure(node.get(), node->get_name(), costmap_ros.get(), model);
  optimizer.on_activate();

  return optimizer;
}

auto getDummyTwist()
{
  geometry_msgs::msg::Twist twist;
  return twist;
}

auto getDummyPointStamped(auto node, std::string frame = std::string("dummy"))
{
  geometry_msgs::msg::PoseStamped point;
  detail::setHeader(point, node, frame);

  return point;
}

auto getDummyPointStamped(auto node, auto x, auto y)
{
  auto point = getDummyPointStamped(node);
  point.pose.position.x = x;
  point.pose.position.y = y;

  return point;
}

auto getDummyPath(auto node, std::string node_name = std::string("dummy"))
{
  nav_msgs::msg::Path path;
  detail::setHeader(path, node, node_name);
  return path;
}

auto getDummyPath(auto points_count, auto node)
{
  auto path = getDummyPath(node);

  for (size_t i = 0; i < points_count; i++) { path.poses.push_back(getDummyPointStamped(node)); }

  return path;
}

auto getIncrementalDummyPath(auto start_x,
  auto start_y,
  auto step_x,
  auto step_y,
  auto points_count,
  auto node)
{
  auto path = getDummyPath(node);

  for (size_t i = 0; i < points_count; i++) {
    auto x = start_x + static_cast<float>(i) * step_x;
    auto y = start_y + static_cast<float>(i) * step_y;
    path.poses.push_back(getDummyPointStamped(node, x, y));
  }

  return path;
}

}// namespace factory
