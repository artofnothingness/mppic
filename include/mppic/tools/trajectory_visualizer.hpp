// Copyright 2022 FastSense, Samsung Research
#ifndef MPPIC__TRAJECTORY_VISUALIZER_HPP_
#define MPPIC__TRAJECTORY_VISUALIZER_HPP_

#include <memory>
#include <string>
#include <xtensor/xtensor.hpp>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "mppic/tools/parameters_handler.hpp"
#include "mppic/models/trajectories.hpp"

namespace mppi
{

class TrajectoryVisualizer
{
public:
  TrajectoryVisualizer() = default;

  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    const std::string & frame_id, ParametersHandler * parameters_handler);
  void on_cleanup();
  void on_activate();
  void on_deactivate();

  void add(const xt::xtensor<float, 2> & trajectory);
  void add(const models::Trajectories & trajectories);
  void visualize(nav_msgs::msg::Path plan);
  void reset();

protected:
  std::string frame_id_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
  trajectories_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_path_pub_;

  std::unique_ptr<visualization_msgs::msg::MarkerArray> points_;
  int marker_id_ = 0;

  ParametersHandler * parameters_handler_;

  size_t trajectory_step_{0};
  size_t time_step_{0};

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // MPPIC__TRAJECTORY_VISUALIZER_HPP_
