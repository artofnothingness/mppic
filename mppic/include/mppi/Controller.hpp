#pragma once

#include "nav2_core/controller.hpp"

#include "rclcpp/rclcpp.hpp"

#include <string>

#include "mppi/Optimizer.hpp"

namespace ultra::mppi {

class Controller : public nav2_core::Controller {

  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Twist = geometry_msgs::msg::Twist;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Path = nav_msgs::msg::Path;

public:
  Controller() = default;
  ~Controller() override = default;

  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr const &parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> const &tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> const
                     &costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  auto computeVelocityCommands(PoseStamped const &pose, Twist const &velocity)
      -> TwistStamped override;

  void setPlan(Path const &path) override;

private:
  void getParams();

  void getParam(auto &param, std::string param_name, auto default_value) {
    auto full_name = node_name_ + '.' + param_name;
    node_->declare_parameter(full_name, rclcpp::ParameterValue(default_value));
    node_->get_parameter(full_name, param);
  }

  // Configure step params
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string node_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  Path global_plan_;

  // Entities
  Optimizer optimizer_;
};

} // namespace ultra::mppi
