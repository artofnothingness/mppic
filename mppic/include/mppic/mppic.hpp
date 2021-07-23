#pragma once

#include <string>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ultra {

class MPPIController : public nav2_core::Controller {
public:
  MPPIController() = default;
  ~MPPIController() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &parent,
                 std::string name, const std::shared_ptr<tf2_ros::Buffer> &tf,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
                     &costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity) override;

  void setPlan(const nav_msgs::msg::Path &path) override;
};

} // namespace ultra
