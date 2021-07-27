#include <string>

#include "mppi/Controller.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace ultra::mppi {

void Controller::configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr const &parent,
    std::string node_name, std::shared_ptr<tf2_ros::Buffer> const &tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> const &costmap_ros) {

  optimizer_ = Optimizer();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  node_name_ = node_name;
  node_ = parent;

  getParams();
}

void Controller::getParams() {
  getParam(optimizer_.model_dt, "model_dt", 0.1);
  getParam(optimizer_.time_steps, "time_steps", 20);
  getParam(optimizer_.batch_size, "batch_size", 100);
  getParam(optimizer_.v_std, "v_std", 0.1);
  getParam(optimizer_.w_std, "w_std", 0.1);
  getParam(optimizer_.v_limit, "v_limit", 0.5);
  getParam(optimizer_.w_limit, "w_limit", 1.0);
  getParam(optimizer_.n_optimizations, "n_optimizations", 2);
  getParam(optimizer_.lookahead_dist, "lookahead_dist", 1.2);

  optimizer_.reset();
}

void Controller::cleanup() {}
void Controller::activate() {}
void Controller::deactivate() {}

void Controller::setPlan(Path const &path) { global_plan_ = path; }

auto Controller::computeVelocityCommands(PoseStamped const &pose,
                                         Twist const &twist) -> TwistStamped {

  (void)pose;
  (void)twist;

  return TwistStamped{};
}

} // namespace ultra::mppi

#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

PLUGINLIB_EXPORT_CLASS(ultra::mppi::Controller, nav2_core::Controller)
