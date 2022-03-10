#include <memory>

#include "mppic/trajectory_visualizer.hpp"

namespace mppi::visualization {

void TrajectoryVisualizer::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & frame_id)
{
  auto node = parent.lock();
  frame_id_ = frame_id;
  trajectories_publisher_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectories", 1);
  transformed_path_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);

  reset();
  RCLCPP_INFO(logger_, "Configured");
}

void TrajectoryVisualizer::on_cleanup()
{
  trajectories_publisher_.reset();
  transformed_path_pub_.reset();
}

void TrajectoryVisualizer::on_activate()
{
  trajectories_publisher_->on_activate();
  transformed_path_pub_->on_activate();
}

void TrajectoryVisualizer::on_deactivate()
{
  trajectories_publisher_->on_deactivate();
  transformed_path_pub_->on_deactivate();
}

void TrajectoryVisualizer::reset()
{
  marker_id_ = 0;
  points_ = std::make_unique<visualization_msgs::msg::MarkerArray>();
}

void TrajectoryVisualizer::visualize(nav_msgs::msg::Path & plan)
{
  trajectories_publisher_->publish(std::move(points_));
  reset();
  std::unique_ptr<nav_msgs::msg::Path> plan_ptr = std::make_unique<nav_msgs::msg::Path>(plan);
  transformed_path_pub_->publish(std::move(plan_ptr));
}

visualization_msgs::msg::Marker TrajectoryVisualizer::createMarker(
  int id, const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color,
  const std::string & frame_id)
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

geometry_msgs::msg::Pose TrajectoryVisualizer::createPose(double x, double y, double z)
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

geometry_msgs::msg::Vector3 TrajectoryVisualizer::createScale(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}

std_msgs::msg::ColorRGBA TrajectoryVisualizer::createColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

} // namespace mppi::visualization
