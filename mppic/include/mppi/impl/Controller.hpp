#pragma once

#include "mppi/Controller.hpp"
#include "mppi/Models.hpp"

#include "utils/common.hpp"
#include "utils/geometry.hpp"

namespace mppi {

template <typename T, typename Model> 
auto Controller<T, Model>::
configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
          std::string node_name,
          const std::shared_ptr<tf2_ros::Buffer> &tf,
          const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros) 
-> void
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  node_name_ = node_name;

  getParams();
  setPublishers();
  createComponents();

  utils::configure(optimizer_, path_handler_, trajectory_visualizer_);
}

template <typename T, typename Model> 
auto Controller<T, Model>::
cleanup() 
-> void
{
  transformed_path_pub_.reset();
  utils::cleanup(optimizer_, path_handler_, trajectory_visualizer_);
}

template <typename T, typename Model> 
auto Controller<T, Model>::
activate() 
-> void
{
  transformed_path_pub_->on_activate();
  utils::activate(optimizer_, path_handler_, trajectory_visualizer_);
}

template <typename T, typename Model> 
auto Controller<T, Model>::
deactivate() 
-> void
{
  transformed_path_pub_->on_deactivate();
  utils::deactivate(optimizer_, path_handler_, trajectory_visualizer_);
}

template <typename T, typename Model> 
auto Controller<T, Model>::
computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose, 
                        const geometry_msgs::msg::Twist &velocity)
-> geometry_msgs::msg::TwistStamped 
{

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  auto &&transformed_plan = path_handler_.transformPath(pose);
  auto &&cmd = optimizer_.evalNextBestControl(pose, velocity, transformed_plan);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Control Loop time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

  if (visualize_) {
    trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), 5, 2);
    trajectory_visualizer_.add(optimizer_.evalTrajectoryFromControlSequence(pose));
    trajectory_visualizer_.visualize();
    trajectory_visualizer_.reset();

    transformed_path_pub_->publish(transformed_plan);
  }

  return cmd;
}

template <typename T, typename Model> 
auto Controller<T, Model>::
getParams() 
-> void
{
  auto getParam = [&](const std::string &param_name, auto default_value) {
    std::string name = node_name_ + '.' + param_name;
    return utils::getParam(name, default_value, parent_);
  };
  visualize_ = getParam("visualize", true);
}

template <typename T, typename Model> 
auto Controller<T, Model>::
setPublishers() 
-> void
{
  transformed_path_pub_ = parent_->create_publisher<nav_msgs::msg::Path>(
      "transformed_global_plan", 1);
}

template <typename T, typename Model> 
auto Controller<T, Model>::
createComponents() 
-> void
{
  auto &model = models::NaiveModel<T>;

  optimizer_ = optimization::Optimizer<T>(parent_, node_name_, costmap_ros_, model);
  path_handler_ =
      handlers::PathHandler(parent_, node_name_, costmap_ros_, tf_buffer_);

  trajectory_visualizer_ = visualization::TrajectoryVisualizer(
      parent_, costmap_ros_->getGlobalFrameID());
}

} // namespace mppi
