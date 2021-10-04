#pragma once

#include "rclcpp/rclcpp.hpp"

#include "nav2_core/controller.hpp"

#include "mppi/impl/Optimizer.hpp"
#include "mppi/impl/PathHandler.hpp"
#include "visualization/TrajectoryVisualizer.hpp"

#include "utils/common.hpp"
#include "utils/geometry.hpp"

namespace mppi {

template<
  typename T,
  typename Model = xt::xtensor<T, 2>(const xt::xtensor<T, 2> &)>
class Controller : public nav2_core::Controller
{

public:
  Controller() = default;

  void configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
    std::string node_name,
    const std::shared_ptr<tf2_ros::Buffer> &tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros) final;

  void cleanup() final;
  void activate() final;
  void deactivate() final;

  auto computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &robot_speed)
    -> geometry_msgs::msg::TwistStamped final;

  void setPlan(const nav_msgs::msg::Path &path) final
  {
    path_handler_.setPath(path);
  }

private:
  void getParams();
  void setPublishers();
  void configureComponents();
  void handleVisualizations(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &robot_speed,
    const nav_msgs::msg::Path &transformed_plan);

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string node_name_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>
    transformed_path_pub_;

  optimization::Optimizer<T, Model> optimizer_;
  handlers::PathHandler path_handler_;
  visualization::TrajectoryVisualizer trajectory_visualizer_;

  bool visualize_;
};

}// namespace mppi
