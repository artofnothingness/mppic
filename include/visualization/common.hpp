#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mppi::visualization {

inline auto createMarker(
  int id,
  const geometry_msgs::msg::Pose &pose,
  const geometry_msgs::msg::Vector3 &scale,
  const std_msgs::msg::ColorRGBA &color,
  const std::string &frame_id)
  -> visualization_msgs::msg::Marker
{

  using visualization_msgs::msg::Marker;
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time(0, 0);
  marker.ns = "MarkerNS";
  marker.id = id;
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;

  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;

  return marker;
}

inline auto createPose(double x, double y, double z)
  -> geometry_msgs::msg::Pose
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  pose.orientation.w = 1;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;

  return pose;
}

inline auto createScale(double x, double y, double z)
  -> geometry_msgs::msg::Vector3
{
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;

  return scale;
}

inline auto createColor(double r, double g, double b, double a)
  -> std_msgs::msg::ColorRGBA
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;

  return color;
}

}// namespace mppi::visualization
