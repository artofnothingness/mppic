#pragma once

#include "mppic/handlers/PathHandlerImpl.hpp"
#include "mppic/optimization/OptimizerImpl.hpp"
#include "mppic/visualization/TrajectoryVisualizer.hpp"

#include <nav2_core/controller.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mppi {
template <typename T>
class Controller : public nav2_core::Controller
{
public:
  Controller() = default;

  void configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & parent, std::string node_name,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) final;

  void cleanup() final;
  void activate() final;
  void deactivate() final;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed) final;

  void
  setPlan(const nav_msgs::msg::Path & path) final
  {
    path_handler_.setPath(path);
  }

private:
  void getParams();
  void setPublishers();
  void configureComponents();

  void handleVisualizations(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    std::unique_ptr<nav_msgs::msg::Path> && transformed_plan);

  std::string node_name_;
  rclcpp_lifecycle::LifecycleNode * parent_{nullptr};
  nav2_costmap_2d::Costmap2DROS * costmap_ros_{nullptr};
  tf2_ros::Buffer * tf_buffer_{nullptr};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_path_pub_;

  optimization::Optimizer<T> optimizer_;
  handlers::PathHandler path_handler_;
  visualization::TrajectoryVisualizer trajectory_visualizer_;

  bool visualize_;
};

}  // namespace mppi
