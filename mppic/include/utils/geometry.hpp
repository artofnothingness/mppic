#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "algorithm"
#include "xtensor/xarray.hpp"

namespace mppi::geometry {

template <typename T, typename H>
geometry_msgs::msg::TwistStamped toTwistStamped(T &&velocities,
                                                const H &header) {
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = header.frame_id;
  twist.header.stamp = header.stamp;
  twist.twist.linear.x = velocities(0);
  twist.twist.angular.z = velocities(1);
  return twist;
}

template <typename T> auto hypot(const T &p1, const T &p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;

  return std::hypot(dx, dy, dz);
}

template <>
inline auto hypot(const geometry_msgs::msg::Pose &lhs,
                  const geometry_msgs::msg::Pose &rhs) {
  return hypot(lhs.position, rhs.position);
}

template <>
inline auto hypot(const geometry_msgs::msg::PoseStamped &lhs,
                  const geometry_msgs::msg::PoseStamped &rhs) {
  return hypot(lhs.pose, rhs.pose);
}

} // namespace mppi::utils::geometry
