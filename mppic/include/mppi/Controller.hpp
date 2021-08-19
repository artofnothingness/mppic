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
          typename Tensor = xt::xarray<T>,
          typename Model = Tensor(const Tensor &)>
class Controller : public nav2_core::Controller {

public:
  Controller() = default;
  ~Controller() override = default;

  void configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
                 std::string node_name,
                 const std::shared_ptr<tf2_ros::Buffer> &tf,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
                     &costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  auto computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                               const geometry_msgs::msg::Twist &velocity)
      -> geometry_msgs::msg::TwistStamped override;

  void setPlan(const nav_msgs::msg::Path &path) override {
    path_handler_.setPath(path);
  }

private:
  void getParams();
  void setPublishers();
  void createComponents();

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string node_name_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>
      transformed_path_pub_;

  optimization::Optimizer<T, Tensor, Model> optimizer_;
  handlers::PathHandler path_handler_;
  visualization::TrajectoryVisualizer trajectory_visualizer_;

  bool visualize_;
};

} // namespace mppi
