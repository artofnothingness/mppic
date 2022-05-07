// Copyright 2022 FastSense, Samsung Research
#ifndef MPPIC__TRAJECTORY_VISUALIZER_HPP_
#define MPPIC__TRAJECTORY_VISUALIZER_HPP_

#include <memory>
#include <string>

// 3rdparty
#include <experimental/mdspan>

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// msgs
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace stdex = std::experimental;

namespace mppi
{

using span2d = stdex::mdspan<double, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>;
using span3d = stdex::mdspan<double, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent, stdex::dynamic_extent>>;

class TrajectoryVisualizer
{
public:
  TrajectoryVisualizer() = default;

  void on_configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & frame_id);
  void on_cleanup();
  void on_activate();
  void on_deactivate();

  void add(const span2d &trajectory);
  void add(const span3d &trajectories, const size_t batch_step,
    const size_t time_step);
  void visualize(nav_msgs::msg::Path plan);
  void reset();

protected:
  std::string frame_id_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
  trajectories_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_path_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>>
  local_goal_publisher_;

  std::unique_ptr<visualization_msgs::msg::MarkerArray> points_;
  int marker_id_ = 0;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // MPPIC__TRAJECTORY_VISUALIZER_HPP_
