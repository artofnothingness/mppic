#pragma once

#include "nav2_core/controller.hpp"

#include "rclcpp/rclcpp.hpp"

#include <string>

#include "mppi/Optimizer.hpp"
#include "mppi/PathHandler.hpp"
#include "mppi/Utils.hpp"

#include "mppi/Models.hpp"

namespace ultra::mppi {

using std::shared_ptr;
using std::string;

using rclcpp_lifecycle::LifecycleNode;
using tf2_ros::Buffer;

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Path;

using nav2_costmap_2d::Costmap2DROS;

template <typename T> class Controller : public nav2_core::Controller {
public:
  using Optimizer = optimization::Optimizer<T>;
  using PathHandler = handlers::PathHandler;

  Controller() = default;
  ~Controller() override = default;

  void configure(const shared_ptr<LifecycleNode> &parent, string node_name,
                 const shared_ptr<Buffer> &tf,
                 const shared_ptr<Costmap2DROS> &costmap_ros) override {

    parent_ = parent;
    costmap_ros_ = costmap_ros;
    tf_buffer_ = tf;
    node_name_ = node_name;

    auto &model = models::NaiveModel<T>;

    optimizer_ =
        Optimizer(parent_, node_name_, tf_buffer_, costmap_ros_, model);

    path_handler_ =
        PathHandler(parent_, node_name_, tf_buffer_, costmap_ros_);
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  auto computeVelocityCommands(const PoseStamped &pose, const Twist &velocity)
      -> TwistStamped override {

    auto transformed_path = path_handler_.transformPath(pose);
    auto control_cmd = optimizer_.evalNextControl(pose, velocity, transformed_path);
    return control_cmd; // TODO
  }

  void setPlan(const Path &path) override { path_handler_.setPlan(path); }

private:
  shared_ptr<LifecycleNode> parent_;
  shared_ptr<Costmap2DROS> costmap_ros_;
  shared_ptr<Buffer> tf_buffer_;
  string node_name_;
  Path global_plan_;

  Optimizer optimizer_;
  PathHandler path_handler_;
};

} // namespace ultra::mppi
