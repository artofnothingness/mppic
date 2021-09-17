#pragma once

#include "visualization/common.hpp"

namespace mppi::visualization
{

class TrajectoryVisualizer
{
public:
  TrajectoryVisualizer() = default;
  auto on_configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & parent,
    const std::string & frame_id)
  ->void
  {
    frame_id_ = frame_id;
    parent_ = parent;

    trajectories_publisher_ =
      parent_->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectories", 1);

    RCLCPP_INFO(logger_, "Configured");
  }

  auto on_cleanup()->void {trajectories_publisher_.reset();}
  auto on_activate()->void {trajectories_publisher_->on_activate();}
  auto on_deactivate()->void {trajectories_publisher_->on_deactivate();}


  auto reset()->void
  {
    marker_id_ = 0;
    points_.markers.clear();
  }


  template<typename Container>
  auto add(Container && trajectory)
  ->void
  {
    auto & size = trajectory.shape()[0];
    if (not size) {
      return;
    }

    for (size_t i = 0; i < size; i++) {
      double blue_component = 1 - static_cast<double>(i) / size;
      double green_component = static_cast<double>(i) / size;

      auto pose = createPose(trajectory(i, 0), trajectory(i, 1), 0.06);
      auto scale = i != size - 1 ? createScale(0.03, 0.03, 0.07) :
        createScale(0.10, 0.10, 0.10);

      auto color = createColor(0, green_component, blue_component, 1);
      auto marker = createMarker(marker_id_++, pose, scale, color, frame_id_);

      points_.markers.push_back(marker);
    }
  }

  template<typename Container>
  auto add(Container && trajectories, double batch_step, double time_step)
  ->void
  {
    if (not trajectories.shape()[0]) {
      return;
    }

    auto & shape = trajectories.shape();

    for (size_t i = 0; i < shape[0]; i += batch_step) {
      for (size_t j = 0; j < shape[1]; j += time_step) {

        double blue_component = 1 - static_cast<double>(j) / shape[1];
        double green_component = static_cast<double>(j) / shape[1];

        auto pose = createPose(trajectories(i, j, 0), trajectories(i, j, 1), 0.03);
        auto scale = createScale(0.03, 0.03, 0.03);
        auto color = createColor(0, green_component, blue_component, 1);
        auto marker = createMarker(marker_id_++, pose, scale, color, frame_id_);

        points_.markers.push_back(marker);
      }
    }
  }

  auto visualize()->void
  {
    trajectories_publisher_->publish(points_);
  }

private:
  std::string frame_id_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
  trajectories_publisher_;

  visualization_msgs::msg::MarkerArray points_;
  int marker_id_ = 0;

  rclcpp::Logger logger_{rclcpp::get_logger("Trajectory Visualizer")};
};

} // namespace mppi::visualization
