#pragma once
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace factory {
using T = float;
namespace detail {
auto
setHeader(auto && msg, auto node, std::string frame)
{
  auto time = node->get_clock()->now();
  msg.header.frame_id = frame;
  msg.header.stamp = time;
}
}  // namespace detail

geometry_msgs::msg::Point
getDummyPoint(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0;

  return point;
}
auto
getDummyCostmapRos()
{
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");
  costmap_ros->on_configure(rclcpp_lifecycle::State{});
  return costmap_ros;
}

std::shared_ptr<nav2_costmap_2d::Costmap2D>
getDummyCostmap(
  unsigned int cells_x, unsigned int cells_y, double origin_x, double origin_y, double resolution,
  unsigned char default_value)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    cells_x, cells_y, resolution, origin_x, origin_y, default_value);

  return costmap;
}

std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
getDummyCostmapRos(
  unsigned int cells_x, unsigned int cells_y, double origin_x, double origin_y, double resolution,
  unsigned char default_value)
{
  auto costmap_ros = factory::getDummyCostmapRos();
  auto costmap_ptr = costmap_ros->getCostmap();
  auto costmap =
    factory::getDummyCostmap(cells_x, cells_y, origin_x, origin_y, resolution, default_value);
  *(costmap_ptr) = *costmap;

  return costmap_ros;
}

auto
getDummyModel()
{
  auto model = mppi::models::NaiveModel<T>;
  return model;
}

auto
getDummyNode(rclcpp::NodeOptions options, std::string node_name = std::string("dummy"))
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);
  return node;
}

auto
getDummyOptimizer(auto node, auto costmap_ros, auto model)
{
  auto optimizer = mppi::optimization::Optimizer<T>();
  optimizer.on_configure(node.get(), node->get_name(), costmap_ros.get(), model);
  optimizer.on_activate();

  return optimizer;
}

auto
getDummyTwist()
{
  geometry_msgs::msg::Twist twist;
  return twist;
}

auto
getDummyPointStamped(auto node, std::string frame = std::string("odom"))
{
  geometry_msgs::msg::PoseStamped point;
  detail::setHeader(point, node, frame);

  return point;
}

auto
getDummyPointStamped(auto node, double x, double y)
{
  auto point = getDummyPointStamped(node);
  point.pose.position.x = x;
  point.pose.position.y = y;

  return point;
}

auto
getDummyPath(auto node, std::string frame = std::string("odom"))
{
  nav_msgs::msg::Path path;
  detail::setHeader(path, node, frame);
  return path;
}

auto
getDummyPath(auto points_count, auto node)
{
  auto path = getDummyPath(node);

  for (size_t i = 0; i < points_count; i++) {
    path.poses.push_back(getDummyPointStamped(node));
  }

  return path;
}

auto
getIncrementalDummyPath(
  double start_x, double start_y, double step_x, double step_y, unsigned int points_count,
  auto node)
{
  auto path = getDummyPath(node);

  for (size_t i = 0; i < points_count; i++) {
    double x = start_x + static_cast<double>(i) * step_x;
    double y = start_y + static_cast<double>(i) * step_y;
    path.poses.push_back(getDummyPointStamped(node, x, y));
  }

  return path;
}

auto
getDummySquareFootprint(double a)
{
  return std::vector{
    getDummyPoint(a, a), getDummyPoint(-a, -a), getDummyPoint(a, -a), getDummyPoint(-a, a)};
}

}  // namespace factory
