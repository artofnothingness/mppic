#pragma once

#include "mppi/Controller.hpp"
#include "mppi/Models.hpp"

#include "utils/common.hpp"
#include "utils/geometry.hpp"
#include "utils/visualization.hpp"

namespace mppi {

template <typename T, typename Tensor, typename Model>
void Controller<T, Tensor, Model>::configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
    std::string node_name, const std::shared_ptr<tf2_ros::Buffer> &tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros) {
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  node_name_ = node_name;

  getParams();
  setPublishers();
  createComponents();

  utils::configure(optimizer_, path_handler_, trajectory_visualizer_);
}

template <typename T, typename Tensor, typename Model>
void Controller<T, Tensor, Model>::cleanup() {
  transformed_path_pub_.reset();
  utils::cleanup(optimizer_, path_handler_, trajectory_visualizer_);
}

template <typename T, typename Tensor, typename Model>
void Controller<T, Tensor, Model>::activate() {
  transformed_path_pub_->on_activate();
  utils::activate(optimizer_, path_handler_, trajectory_visualizer_);
}

template <typename T, typename Tensor, typename Model>
void Controller<T, Tensor, Model>::deactivate() {
  transformed_path_pub_->on_deactivate();
  utils::deactivate(optimizer_, path_handler_, trajectory_visualizer_);
}

template <typename T, typename Tensor, typename Model>
auto Controller<T, Tensor, Model>::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity)
    -> geometry_msgs::msg::TwistStamped {

  auto &&transformed_plan = path_handler_.transformPath(pose);
  auto &&cmd = optimizer_.evalNextControl(velocity, transformed_plan);

  if (visualize_) {
    transformed_path_pub_->publish(transformed_plan);
    trajectory_visualizer_.visualize(optimizer_.getTrajectories(), 10, 2);
  }

  return cmd;
}

template <typename T, typename Tensor, typename Model>
void Controller<T, Tensor, Model>::getParams() {
  auto getParam = [&](const std::string &param_name, auto default_value) {
    std::string name = node_name_ + '.' + param_name;
    return utils::getParam(name, default_value, parent_);
  };
  visualize_ = getParam("visualize", true);
}

template <typename T, typename Tensor, typename Model>
void Controller<T, Tensor, Model>::setPublishers() {
  transformed_path_pub_ = parent_->create_publisher<nav_msgs::msg::Path>(
      "transformed_global_plan", 1);
}

template <typename T, typename Tensor, typename Model>
void Controller<T, Tensor, Model>::createComponents() {
  auto &model = models::NaiveModel<T>;
  auto costmap = costmap_ros_->getCostmap();

  optimizer_ = optimization::Optimizer<T>(parent_, node_name_, costmap, model);
  path_handler_ =
      handlers::PathHandler(parent_, node_name_, costmap_ros_, tf_buffer_);

  trajectory_visualizer_ = visualization::TrajectoryVisualizer(
      parent_, costmap_ros_->getBaseFrameID());
}

} // namespace mppi
