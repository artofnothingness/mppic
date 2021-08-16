#pragma once

#include "visualization/common.hpp"
#include "xtensor/xarray.hpp"

namespace mppi::visualization {

class TrajectoryVisualizer {
public:
  TrajectoryVisualizer() = default;

  TrajectoryVisualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
                       const std::string &frame_id)
      : frame_id_(frame_id), parent_(parent) {}

  void on_configure() {
    trajectories_publisher_ =
        parent_->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectories", 1);

    RCLCPP_INFO(logger_, "Configured");
  }

  void on_cleanup() { trajectories_publisher_.reset(); }
  void on_activate() { trajectories_publisher_->on_activate(); }
  void on_deactivate() { trajectories_publisher_->on_deactivate(); }

  template <typename Container>
  void visualize(Container &&trajectories, double batch_step, double time_step) {

    if (not trajectories.shape()[0])
      return;

    visualization_msgs::msg::MarkerArray points;
    auto &shape = trajectories.shape();

    int marker_id = 0;
    for (size_t i = 0; i < shape[0]; i += batch_step) {
      for (size_t j = 0; j < shape[1]; j += time_step) {

        double blue_component = 1 - static_cast<double>(j) / shape[1];
        double green_component = static_cast<double>(j) / shape[1];
        /* double red_component = static_cast<double>(j) / shape[1]; */

        auto pose = createPose(trajectories(i, j, 0), trajectories(i, j, 1), 0.03);
        auto scale = createScale(0.03, 0.03, 0.03);
        auto color = createColor(0, green_component, blue_component, 1);
        auto marker = createMarker(marker_id++, pose, scale, color, frame_id_);

        points.markers.push_back(marker);
      }
    }

    trajectories_publisher_->publish(points);
  }

private:
  std::string frame_id_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>

      trajectories_publisher_;
  rclcpp::Logger logger_{rclcpp::get_logger("Trajectory Visualizer")};
};

} // namespace mppi::visualization
