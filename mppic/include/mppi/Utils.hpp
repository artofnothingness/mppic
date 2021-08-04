#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "xtensor/xarray.hpp"
#include "algorithm"

namespace ultra::mppi::utils {

using geometry_msgs::msg::TwistStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Pose;

using rclcpp_lifecycle::LifecycleNode;
using std::string;

template <typename T>
void getParam(string const &param_name, T default_value,
              LifecycleNode::SharedPtr node, T &param) {

  node->declare_parameter(param_name, rclcpp::ParameterValue(default_value));
  node->get_parameter(param_name, param);
}

TwistStamped toTwistStamped(auto const &velocities, auto const &header) {
  TwistStamped twist;
  twist.header.frame_id = header.frame_id;
  twist.header.stamp = header.stamp;
  twist.twist.linear.x = velocities[0];
  twist.twist.angular.z = velocities[1];
  return twist;
}


template <typename T>
auto hypot(T const &p1, T const &p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;

  return std::hypot(dx, dy, dz);
}

template<>
auto hypot(Pose const &lhs, Pose const &rhs) {
  auto const &p1 = rhs.position;
  auto const &p2 = lhs.position;

  return hypot(p1, p2);
}

template<>
auto hypot(PoseStamped const &lhs, PoseStamped const &rhs) {
  auto const &p1 = rhs.pose;
  auto const &p2 = lhs.pose;

  return hypot(p1, p2);
}




} // namespace ultra::mppi::utils
