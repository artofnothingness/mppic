// Copyright 2022 FastSense, Samsung Research
#include <stdint.h>
#include <chrono>
#include "mppic/controller.hpp"
#include "mppic/motion_models.hpp"
#include "mppic/utils.hpp"

namespace mppi
{

void Controller::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent);

  auto node = parent_.lock();
  clock_ = node->get_clock();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false, ParameterType::Dynamic);

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(parent_, costmap_ros_->getGlobalFrameID());

  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void Controller::cleanup()
{
  trajectory_visualizer_.on_cleanup();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void Controller::activate()
{
  trajectory_visualizer_.on_activate();
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void Controller::deactivate()
{
  trajectory_visualizer_.on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{

  auto t_start = std::chrono::high_resolution_clock::now();
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);
  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal_checker);
  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "Control evaluation time %f", elapsed_time_ms);

  visualize(robot_pose, robot_speed, std::move(transformed_plan));
  return cmd;
}

void Controller::visualize(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav_msgs::msg::Path transformed_plan)
{
  if (!visualize_) {
    return;
  }

  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), 5, 2);
  trajectory_visualizer_.add(optimizer_.evalTrajectoryFromControlSequence(robot_pose, robot_speed));
  trajectory_visualizer_.visualize(transformed_plan);
}

void Controller::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

void Controller::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace mppi

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mppi::Controller, nav2_core::Controller)
