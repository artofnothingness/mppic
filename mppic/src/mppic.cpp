#include <string>

#include "mppic/mppic.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace ultra {

void MPPIController::configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &parent, std::string name,
    const std::shared_ptr<tf2_ros::Buffer> &tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros) {
  (void)parent;
  (void)name;
  (void)tf;
  (void)costmap_ros;
}

void MPPIController::cleanup() {}

void MPPIController::activate() {}

void MPPIController::deactivate() {}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &twist) {
  (void)pose;
  (void)twist;

  return geometry_msgs::msg::TwistStamped{};
}

void MPPIController::setPlan(const nav_msgs::msg::Path &path) { (void)path; }

} // namespace ultra

#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

PLUGINLIB_EXPORT_CLASS(ultra::MPPIController, nav2_core::Controller)
