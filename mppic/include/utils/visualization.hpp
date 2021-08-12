#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "xtensor/xarray.hpp"

namespace mppi::visualization {

namespace details {

inline visualization_msgs::msg::Marker
createMarker(int id, geometry_msgs::msg::Pose pose,
             geometry_msgs::msg::Vector3 scale, std_msgs::msg::ColorRGBA color,
             const std::string &frame_id, const rclcpp::Time &time) {

  using visualization_msgs::msg::Marker;

  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = time;
  marker.ns = "MarkerNS";
  marker.id = id;
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;

  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;

  return marker;
}

inline geometry_msgs::msg::Pose createPose(double x, double y, double z) {
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

inline geometry_msgs::msg::Vector3 createScale(double x, double y, double z) {
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;

  return scale;
}

inline std_msgs::msg::ColorRGBA createColor(double r, double g, double b,
                                            double a) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;

  return color;
}

} // namespace details

class TrajectoryVisualizer {
public:
  TrajectoryVisualizer() = default;

  TrajectoryVisualizer(
      const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
      const std::string &frame_id) {
    parent_ = parent;
    frame_id_ = frame_id;
  };

  void on_configure() {
    trajectories_publisher_ =
        parent_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/trajectories", 1);

    RCLCPP_INFO(logger_, "Configured");
  }

  void on_cleanup() { trajectories_publisher_.reset(); }
  void on_activate() { trajectories_publisher_->on_activate(); }
  void on_deactivate() { trajectories_publisher_->on_deactivate(); }

  template <typename Container>
  void visualize(Container &&trajectories, double batch_step,
                 double time_step) {
    visualization_msgs::msg::MarkerArray points;
    auto &shape = trajectories.shape();

    int marker_id = 0;
    for (size_t i = 0; i < shape[0]; i += batch_step) {
      for (size_t j = 0; j < shape[1]; j += time_step) {

        double blue_component = static_cast<double>(j) / shape[1];
        double red_component = 1 - (static_cast<double>(j) / shape[1]);

        auto pose = details::createPose(trajectories(i, j, 0),
                                        trajectories(i, j, 1), 0);
        auto scale = details::createScale(0.03, 0.03, 0.5);
        auto color = details::createColor(red_component, 0, blue_component, 1);
        auto time = parent_->get_clock()->now();
        auto marker = details::createMarker(marker_id++, pose, scale, color,
                                            frame_id_, time);

        points.markers.push_back(std::move(marker));
      }
    }

    trajectories_publisher_->publish(points);
  }

private:
  std::string frame_id_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>>

      trajectories_publisher_;
  rclcpp::Logger logger_{rclcpp::get_logger("Trajectory Visualizer")};
};

} // namespace mppi::visualization
