#pragma once

#include "nav2_core/controller.hpp"

#include "rclcpp/rclcpp.hpp"

#include <string>

#include "mppi/Optimizer.hpp"
#include "mppi/Utils.hpp"

#include "mppi/Models.hpp"


namespace ultra::mppi {

using std::string;
using std::shared_ptr;

using tf2_ros::Buffer;
using rclcpp_lifecycle::LifecycleNode;

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TwistStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Path;

using nav2_costmap_2d::Costmap2DROS;

template<typename T>
class Controller : public nav2_core::Controller {

  using Optimizer = optimization::Optimizer<T>;

public:
  Controller() = default;
  ~Controller() override = default;

  void configure(LifecycleNode::SharedPtr const& parent,
                 string node_name, 
                 shared_ptr<Buffer> const& tf,
                 shared_ptr<Costmap2DROS> const& costmap_ros) override {

    auto &model = models::NaiveModel<T>;

    optimizer_ = Optimizer(parent, node_name, tf, costmap_ros, model);
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  auto computeVelocityCommands(PoseStamped const& pose, Twist const& velocity)
  -> TwistStamped override {
    return optimizer_.evalNextControl(pose, velocity);
  }

  void setPlan(Path const &path) override { optimizer_.setPlan(path); }

private:
  LifecycleNode::SharedPtr parent_;
  std::string node_name_;

  Optimizer optimizer_;
};

} // namespace ultra::mppi
