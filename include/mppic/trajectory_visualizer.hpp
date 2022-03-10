#pragma once

#include <memory>
#include <xtensor/xtensor.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/path.hpp"

namespace mppi
{

class TrajectoryVisualizer
{
public:
  TrajectoryVisualizer() = default;

  void on_configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & frame_id);
  void on_cleanup();
  void on_activate();
  void on_deactivate();

  void reset();

  void add(xt::xtensor<double, 2> trajectory);

  void add(xt::xtensor<double, 3> trajectories, size_t batch_step, size_t time_step);

  void visualize(nav_msgs::msg::Path & plan);

protected:
  visualization_msgs::msg::Marker createMarker(
    int id, const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Vector3 & scale,
    const std_msgs::msg::ColorRGBA & color,
    const std::string & frame_id);

  geometry_msgs::msg::Pose createPose(double x, double y, double z);

  geometry_msgs::msg::Vector3 createScale(double x, double y, double z);

  std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a);

  std::string frame_id_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
    trajectories_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_path_pub_;

  std::unique_ptr<visualization_msgs::msg::MarkerArray> points_;
  int marker_id_ = 0;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPI Trajectory Visualizer")};
};

} // namespace mppi
