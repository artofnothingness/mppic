#pragma once

#include "mppic/PathHandler.hpp"
#include "mppic/optimization/Optimizer.hpp"
#include "mppic/visualization/TrajectoryVisualizer.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mppi {
class Controller : public nav2_core::Controller
{
public:
  Controller() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string node_name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:

  void handleVisualizations(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav_msgs::msg::Path & transformed_plan);

  std::string node_name_;
  rclcpp_lifecycle::LifecycleNode * parent_;
  nav2_costmap_2d::Costmap2DROS * costmap_ros_{nullptr};
  tf2_ros::Buffer * tf_buffer_{nullptr};

  optimization::Optimizer optimizer_;
  handlers::PathHandler path_handler_;
  visualization::TrajectoryVisualizer trajectory_visualizer_;

  bool visualize_;
};

} // namespace mppi
