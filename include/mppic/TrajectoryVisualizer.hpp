#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mppi::visualization {
class TrajectoryVisualizer
{
public:
  TrajectoryVisualizer() = default;

  void on_configure(rclcpp_lifecycle::LifecycleNode * const parent, const std::string & frame_id)
  {
    frame_id_ = frame_id;
    parent_ = parent;

    trajectories_publisher_ =
      parent_->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectories", 1);
    transformed_path_pub_ =
      parent_->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);

    reset();
    RCLCPP_INFO(logger_, "Configured");
  }

  void on_cleanup()
  {
    trajectories_publisher_.reset();
    transformed_path_pub_.reset();
  }

  void on_activate()
  {
    trajectories_publisher_->on_activate();
    transformed_path_pub_->on_activate();
  }

  void on_deactivate()
  {
    trajectories_publisher_->on_deactivate();
    transformed_path_pub_->on_deactivate();
  }

  void reset()
  {
    marker_id_ = 0;
    points_ = std::make_unique<visualization_msgs::msg::MarkerArray>();
  }

  template <typename Container>
  void add(Container && trajectory)
  {
    auto & size = trajectory.shape()[0];
    if (!size) {
      return;
    }

    for (size_t i = 0; i < size; i++) {
      float red_component = static_cast<float>(i) / static_cast<float>(size);

      auto pose = createPose(trajectory(i, 0), trajectory(i, 1), 0.06);
      auto scale = i != size - 1 ? createScale(0.03, 0.03, 0.07) : createScale(0.10, 0.10, 0.10);

      auto color = createColor(red_component, 0, 0, 1);
      auto marker = createMarker(marker_id_++, pose, scale, color, frame_id_);

      points_->markers.push_back(std::move(marker));
    }
  }

  template <typename Container>
  void add(Container && trajectories, size_t batch_step, size_t time_step)
  {
    if (!trajectories.shape()[0]) {
      return;
    }

    auto & shape = trajectories.shape();

    for (size_t i = 0; i < shape[0]; i += batch_step) {
      for (size_t j = 0; j < shape[1]; j += time_step) {
        float blue_component = 1.0f - static_cast<float>(j) / static_cast<float>(shape[1]);
        float green_component = static_cast<float>(j) / static_cast<float>(shape[1]);

        auto pose = createPose(trajectories(i, j, 0), trajectories(i, j, 1), 0.03);
        auto scale = createScale(0.03, 0.03, 0.03);
        auto color = createColor(0, green_component, blue_component, 1);
        auto marker = createMarker(marker_id_++, pose, scale, color, frame_id_);

        points_->markers.push_back(std::move(marker));
      }
    }
  }

  void visualize(nav_msgs::msg::Path & plan)
  {
    trajectories_publisher_->publish(std::move(points_));
    reset();
    std::unique_ptr<nav_msgs::msg::Path> plan_ptr = std::make_unique<nav_msgs::msg::Path>(plan);
    transformed_path_pub_->publish(std::move(plan_ptr));
  }

private:

  visualization_msgs::msg::Marker createMarker(
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

  geometry_msgs::msg::Pose createPose(double x, double y, double z)
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

  geometry_msgs::msg::Vector3 createScale(double x, double y, double z)
  {
    geometry_msgs::msg::Vector3 scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;
    return scale;
  }

  std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a)
  {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

  std::string frame_id_;
  rclcpp_lifecycle::LifecycleNode * parent_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
    trajectories_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_path_pub_;

  std::unique_ptr<visualization_msgs::msg::MarkerArray> points_;
  int marker_id_ = 0;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPI Trajectory Visualizer")};
};

} // namespace mppi::visualization
