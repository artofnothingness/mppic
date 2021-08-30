#pragma once

#include "rclcpp/rclcpp.hpp"

#include "nav2_core/controller.hpp"

#include "mppi/impl/Optimizer.hpp"
#include "mppi/impl/PathHandler.hpp"
#include "visualization/TrajectoryVisualizer.hpp"

#include "utils/common.hpp"
#include "utils/geometry.hpp"

namespace mppi {

template <typename T,
          typename Model = xt::xtensor<T, 2>(const xt::xtensor<T, 2> &)>
class Controller : public nav2_core::Controller {

public:
  Controller() = default;
  ~Controller() override = default;

  auto configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
                 std::string node_name,
                 const std::shared_ptr<tf2_ros::Buffer> &tf,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros) 
  -> void override;

  auto cleanup() -> void override;
  auto activate() -> void override;
  auto deactivate() -> void override;

  auto computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                               const geometry_msgs::msg::Twist &velocity)
  -> geometry_msgs::msg::TwistStamped final;

  auto setPlan(const nav_msgs::msg::Path &path) 
  -> void final 
  {
    path_handler_.setPath(path);
  }

private:
  auto getParams() -> void;
  auto setPublishers() -> void;
  auto createComponents() -> void;

private:
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

} // namespace mppi
