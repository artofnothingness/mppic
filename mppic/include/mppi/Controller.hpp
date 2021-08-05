#pragma once

#include "nav2_core/controller.hpp"
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "mppi/Models.hpp"
#include "mppi/Optimizer.hpp"
#include "mppi/PathHandler.hpp"
#include "mppi/Utils.hpp"

namespace ultra::mppi {

using std::shared_ptr;
using std::string;

using rclcpp_lifecycle::LifecycleNode;
using tf2_ros::Buffer;

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Path;

using nav2_costmap_2d::Costmap2D;
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
    costmap_ = costmap_ros_->getCostmap();
    tf_buffer_ = tf;
    node_name_ = node_name;

    global_path_pub_ =
        parent->create_publisher<Path>("transformed_global_plan", 1);

    auto getParam = [&](const string &param_name, auto default_value) {
      string name = node_name + '.' + param_name;
      return utils::getParam(name, default_value, parent_);
    };

    optimizer_ = [&] {
      auto &model = models::NaiveModel<T>;

      double model_dt = getParam("model_dt", 0.1);
      int time_steps = getParam("time_steps", 20);
      int batch_size = getParam("batch_size", 300);
      double std_v = getParam("std_v", 0.1);
      double std_w = getParam("std_w", 0.3);
      double limit_v = getParam("limit_v", 0.3);
      double limit_w = getParam("limit_w", 1.0);
      int iteration_count = getParam("iteration_count", 5);
      double temperature = getParam("temperature", 0.25);
      return Optimizer(batch_size, std_v, std_w, limit_v, limit_w, model_dt,
                       time_steps, iteration_count, temperature, model);
    }();

    path_handler_ = [&] {
      double lookagead_dist = getParam("lookahead_dist", 1.2);
      double transform_tolerance = getParam("transform_tolerance", 1.2);
      return PathHandler(lookagead_dist, transform_tolerance, tf);
    }();
  }

  void cleanup() override { global_path_pub_.reset(); }
  void activate() override { global_path_pub_->on_activate(); }

  void deactivate() override { global_path_pub_->on_deactivate(); }

  auto computeVelocityCommands(const PoseStamped &pose, const Twist &velocity)
      -> TwistStamped override {

    (void)velocity;
    (void)pose;
    auto transformed_plan = path_handler_.transformPath(
        pose, costmap_ros_->getBaseFrameID(), getMaxTransformDistance());

    global_path_pub_->publish(transformed_plan);

    auto cmd = optimizer_.evalNextControl(pose, velocity,
                                          transformed_plan, *costmap_);

    return cmd;
  }

  void setPlan(const Path &path) override { path_handler_.setPath(path); }

private:
  double getMaxTransformDistance() {
    return std::max(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()) *
           costmap_->getResolution() / 2.0;
  }

private:
  shared_ptr<LifecycleNode> parent_;
  shared_ptr<Costmap2DROS> costmap_ros_;
  Costmap2D *costmap_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<Path>> global_path_pub_;
  shared_ptr<Buffer> tf_buffer_;
  string node_name_;

  Optimizer optimizer_;
  PathHandler path_handler_;
};

} // namespace ultra::mppi
