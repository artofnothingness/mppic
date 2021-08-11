#pragma once

#include <chrono>

#include "nav2_core/controller.hpp"
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "mppi/Models.hpp"
#include "mppi/Optimizer.hpp"
#include "mppi/PathHandler.hpp"

#include "utils/common.hpp"
#include "utils/geometry.hpp"
#include "utils/visualization.hpp"

namespace ultra::mppi {

using std::shared_ptr;
using std::string;

using rclcpp_lifecycle::LifecycleNode;
using rclcpp_lifecycle::LifecyclePublisher;
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
  using TrajectoryVisualizer = utils::visualization::TrajectoryVisualizer;

  Controller() = default;
  ~Controller() override = default;

  void configure(const shared_ptr<LifecycleNode> &parent, string node_name,
                 const shared_ptr<Buffer> &tf,
                 const shared_ptr<Costmap2DROS> &costmap_ros) override {

    parent_ = parent;
    costmap_ros_ = costmap_ros;
    tf_buffer_ = tf;
    node_name_ = node_name;

    getParams();
    setPublishers();
    createComponents();

    utils::common::configure(optimizer_, path_handler_, trajectory_visualizer_);
  }

  void cleanup() override {
    transformed_path_pub_.reset();
    utils::common::cleanup(optimizer_, path_handler_, trajectory_visualizer_);
  }
  void activate() override {
    transformed_path_pub_->on_activate();
    utils::common::activate(optimizer_, path_handler_, trajectory_visualizer_);
  }
  void deactivate() override {
    transformed_path_pub_->on_deactivate();
    utils::common::deactivate(optimizer_, path_handler_,
                              trajectory_visualizer_);
  }

  auto computeVelocityCommands(const PoseStamped &pose, const Twist &velocity)
      -> TwistStamped override {
    auto &&transformed_plan = path_handler_.transformPath(pose);
    auto &&cmd = optimizer_.evalNextControl(velocity, transformed_plan);

    if (visualize_) {
      transformed_path_pub_->publish(transformed_plan);
      trajectory_visualizer_.visualize(optimizer_.getTrajectories(), 10, 2);
    }

    return cmd;
  }

  void setPlan(const Path &path) override { path_handler_.setPath(path); }

private:
  void getParams() {
    auto getParam = [&](const string &param_name, auto default_value) {
      string name = node_name_ + '.' + param_name;
      return utils::common::getParam(name, default_value, parent_);
    };
    visualize_ = getParam("visualize", true);
  }

  void setPublishers() {
    transformed_path_pub_ =
        parent_->create_publisher<Path>("transformed_global_plan", 1);
  }

  void createComponents() {
    auto &model = models::NaiveModel<T>;
    auto costmap = costmap_ros_->getCostmap();

    optimizer_ = Optimizer(parent_, node_name_, costmap, model);
    path_handler_ = PathHandler(parent_, node_name_, costmap_ros_, tf_buffer_);

    RCLCPP_INFO(parent_->get_logger(), "FRAME %s ",
                costmap_ros_->getBaseFrameID().c_str());

    trajectory_visualizer_ =
        TrajectoryVisualizer(parent_, costmap_ros_->getBaseFrameID());
  }

private:
  shared_ptr<LifecycleNode> parent_;
  shared_ptr<Costmap2DROS> costmap_ros_;
  shared_ptr<Buffer> tf_buffer_;
  string node_name_;

  bool visualize_;
  shared_ptr<LifecyclePublisher<Path>> transformed_path_pub_;

  Optimizer optimizer_;
  PathHandler path_handler_;
  TrajectoryVisualizer trajectory_visualizer_;
};

} // namespace ultra::mppi
