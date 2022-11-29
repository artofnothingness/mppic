// Copyright 2022 FastSense, Samsung Research
#ifndef MPPIC__TOOLS__TRAJECTORY_VISUALIZER_HPP_
#define MPPIC__TOOLS__TRAJECTORY_VISUALIZER_HPP_

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

/**
 * @class mppi::TrajectoryVisualizer
 * @brief Visualizes trajectories for debugging
 */
class TrajectoryVisualizer
{
public:
  /**
    * @brief Constructor for mppi::TrajectoryVisualizer
    */
  TrajectoryVisualizer() = default;

  /**
    * @brief Configure trajectory visualizer
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param frame_id Frame to publish trajectories in
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    const std::string & frame_id, ParametersHandler * parameters_handler);

  /**
    * @brief Cleanup object on shutdown
    */
  void on_cleanup();

  /**
    * @brief Activate object
    */
  void on_activate();

  /**
    * @brief Deactivate object
    */
  void on_deactivate();

  /**
    * @brief Add an optimal trajectory to visualize
    * @param trajectory Optimal trajectory
    */
  void add(const xt::xtensor<float, 2> & trajectory);

  /**
    * @brief Add candidate trajectories to visualize
    * @param trajectories Candidate trajectories
    */
  void add(const models::Trajectories & trajectories);

  /**
    * @brief Visualize the plan
    * @param plan Plan to visualize
    */
  void visualize(const nav_msgs::msg::Path & plan);

  /**
    * @brief Reset object
    */
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

#endif  // MPPIC__TOOLS__TRAJECTORY_VISUALIZER_HPP_
