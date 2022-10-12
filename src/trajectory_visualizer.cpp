// Copyright 2022 FastSense, Samsung Research
#include <memory>

#include "mppic/tools/trajectory_visualizer.hpp"

namespace mppi
{

namespace
{

inline geometry_msgs::msg::Pose createPose(double x, double y, double z)
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

inline geometry_msgs::msg::Vector3 createScale(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}

inline std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

inline visualization_msgs::msg::Marker createMarker(
  int id, const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color, const std::string & frame_id)
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

}  // namespace

void TrajectoryVisualizer::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  const std::string & frame_id, ParametersHandler * parameters_handler)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  frame_id_ = frame_id;
  trajectories_publisher_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectories", 1);
  transformed_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
  parameters_handler_ = parameters_handler;

  auto getParam = parameters_handler->getParamGetter(name + ".TrajectoryVisualizer");

  getParam(trajectory_step_, "trajectory_step", 5);
  getParam(time_step_, "time_step", 3);

  reset();
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

void TrajectoryVisualizer::add(const xt::xtensor<float, 2> & trajectory)
{
  auto & size = trajectory.shape()[0];
  if (!size) {
    return;
  }

  auto add_marker = [&](auto i) {
      float component = static_cast<float>(i) / static_cast<float>(size);

      auto pose = createPose(trajectory(i, 0), trajectory(i, 1), 0.06);
      auto scale = i != size - 1 ? createScale(0.03, 0.03, 0.07) : createScale(0.07, 0.07, 0.09);
      auto color = createColor(0, component, component, 1);
      auto marker = createMarker(marker_id_++, pose, scale, color, frame_id_);
      points_->markers.push_back(marker);
    };

  for (size_t i = 0; i < size; i++) {
    add_marker(i);
  }
}

void TrajectoryVisualizer::add(
  const models::Trajectories & trajectories)
{
  auto & shape = trajectories.x.shape();
  const float shape_1 = static_cast<float>(shape[1]);
  points_->markers.reserve(floor(shape[0] / trajectory_step_) * floor(shape[1] * time_step_));

  for (size_t i = 0; i < shape[0]; i += trajectory_step_) {
    for (size_t j = 0; j < shape[1]; j += time_step_) {
      const float j_flt = static_cast<float>(j);
      float blue_component = 1.0f - j_flt / shape_1;
      float green_component = j_flt / shape_1;

      auto pose = createPose(trajectories.x(i, j), trajectories.y(i, j), 0.03);
      auto scale = createScale(0.03, 0.03, 0.03);
      auto color = createColor(0, green_component, blue_component, 1);
      auto marker = createMarker(marker_id_++, pose, scale, color, frame_id_);

      points_->markers.push_back(marker);
    }
  }
}

void TrajectoryVisualizer::reset()
{
  marker_id_ = 0;
  points_ = std::make_unique<visualization_msgs::msg::MarkerArray>();
}

void TrajectoryVisualizer::visualize(const nav_msgs::msg::Path & plan)
{
  if (trajectories_publisher_->get_subscription_count() > 0) {
    trajectories_publisher_->publish(std::move(points_));
  }

  reset();

  if (transformed_path_pub_->get_subscription_count() > 0) {
    auto plan_ptr = std::make_unique<nav_msgs::msg::Path>(plan);
    transformed_path_pub_->publish(std::move(plan_ptr));
  }
}

}  // namespace mppi
