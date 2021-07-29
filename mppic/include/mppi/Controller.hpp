#pragma once

#include "mppi/Optimizer.hpp"

#include "nav2_core/controller.hpp"

#include "rclcpp/rclcpp.hpp"

#include <string>


namespace ultra::mppi {

template<typename T>
class Controller : public nav2_core::Controller {

  using Optimizer = optimization::Optimizer<T>;

  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Twist = geometry_msgs::msg::Twist;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Path = nav_msgs::msg::Path;
  using Costmap2DROS = nav2_costmap_2d::Costmap2DROS;
  using TfBuffer = tf2_ros::Buffer;
  using ManagedNode = rclcpp_lifecycle::LifecycleNode;

public:
  Controller() = default;
  ~Controller() override = default;

  void configure(ManagedNode::SharedPtr const &parent,
                 std::string node_name, 
                 std::shared_ptr<TfBuffer> const &tf,
                 std::shared_ptr<Costmap2DROS> const &costmap_ros) override {


    optimizer_ = Optimizer();
    costmap_ros_ = costmap_ros;
    tf_ = tf;
    node_name_ = node_name;
    node_ = parent;

    getParams();
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  auto computeVelocityCommands(PoseStamped const &pose, Twist const &velocity)
  -> TwistStamped override {

    (void)velocity;
    (void)pose;
    return TwistStamped{};
  }

  void setPlan(Path const &path) override { global_plan_ = path; }

private:
  void getParams() {
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

  void getParam(auto &param, std::string const& param_name, auto default_value) {
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
