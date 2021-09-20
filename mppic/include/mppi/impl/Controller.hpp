#pragma once

#include "mppi/Controller.hpp"
#include "mppi/Models.hpp"

#include "utils/common.hpp"
#include "utils/geometry.hpp"

namespace mppi
{

template<typename T, typename Model>
void Controller<T, Model>::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & parent,
  std::string node_name,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  node_name_ = node_name;

  getParams();
  setPublishers();
  configureComponents();
}

template<typename T, typename Model>
void Controller<T, Model>::cleanup()
{
  optimizer_.on_cleanup();
  path_handler_.on_cleanup();
  trajectory_visualizer_.on_cleanup();
  
  transformed_path_pub_.reset();
}

template<typename T, typename Model>
void Controller<T, Model>::activate()
{
  transformed_path_pub_->on_activate();

  optimizer_.on_activate();
  path_handler_.on_activate();
  trajectory_visualizer_.on_activate();
}

template<typename T, typename Model>
void Controller<T, Model>::deactivate()
{
  transformed_path_pub_->on_deactivate();
  optimizer_.on_deactivate();
  path_handler_.on_deactivate();
  trajectory_visualizer_.on_deactivate();

}

template<typename T, typename Model>
auto Controller<T, Model>::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed)
->geometry_msgs::msg::TwistStamped
{
  auto && transformed_plan = path_handler_.transformPath(robot_pose);
  auto && cmd = optimizer_.evalNextBestControl(
    robot_pose, robot_speed, transformed_plan);

  if (visualize_) {
    handleVisualizations(robot_pose, robot_speed, transformed_plan);
  }

  return cmd;
}

template<typename T, typename Model>
void Controller<T, Model>::handleVisualizations(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & transformed_plan)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), 5, 2);
  trajectory_visualizer_.add(optimizer_.evalTrajectoryFromControlSequence(robot_pose, robot_speed));
  trajectory_visualizer_.visualize();
  trajectory_visualizer_.reset();
  transformed_path_pub_->publish(transformed_plan);
}

template<typename T, typename Model>
void Controller<T, Model>::getParams()
{
  auto getParam = [&](const std::string & param_name, auto default_value) {
      std::string name = node_name_ + '.' + param_name;
      return utils::getParam(name, default_value, parent_);
    };
  visualize_ = getParam("visualize", true);
}

template<typename T, typename Model>
void Controller<T, Model>::setPublishers()
{
  transformed_path_pub_ = parent_->create_publisher<nav_msgs::msg::Path>(
    "transformed_global_plan", 1);
}

template<typename T, typename Model>
void Controller<T, Model>::configureComponents()
{
  auto & model = models::NaiveModel<T>;

  optimizer_.on_configure(parent_, node_name_, costmap_ros_, model);
  path_handler_.on_configure(parent_, node_name_, costmap_ros_, tf_buffer_);
  trajectory_visualizer_.on_configure(parent_, costmap_ros_->getGlobalFrameID());
}

} // namespace mppi
