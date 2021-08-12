#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "xtensor/xarray.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor/xslice.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "utils/common.hpp"
#include "utils/geometry.hpp"

namespace mppi::optimization {

template <typename T, typename Tensor = xt::xarray<T>,
          typename Model = Tensor(const Tensor &)>
class Optimizer {

public:
  Optimizer() = default;
  ~Optimizer() = default;

  Optimizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
            const std::string &node_name, nav2_costmap_2d::Costmap2D *costmap,
            Model &&model)
      : model_(model) {

    node_name_ = node_name;
    parent_ = parent;
    costmap_ = costmap;
  }

  void on_configure() {
    getParams();
    resetBatches();
    RCLCPP_INFO(logger_, "Configured");
  }

  void on_cleanup(){};
  void on_activate(){};
  void on_deactivate(){};

  auto evalNextControl(const geometry_msgs::msg::Twist &twist,
                       const nav_msgs::msg::Path &path)
      -> geometry_msgs::msg::TwistStamped {
    static Tensor costs;

    for (int i = 0; i < iteration_count_; ++i) {
      trajectories_ = generateNoisedTrajectoryBatches(twist);
      costs = evalBatchesCosts(trajectories_, path);
      updateControlSequence(costs);
    }

    return getControlFromSequence(path.header);
  }

  Tensor getTrajectories() { return trajectories_; }

private:
  void getParams() {
    auto getParam = [&](const std::string &param_name, auto default_value) {
      std::string name = node_name_ + '.' + param_name;
      return utils::getParam(name, default_value, parent_);
    };

    model_dt_ = getParam("model_dt", 0.1);
    time_steps_ = getParam("time_steps", 20);
    batch_size_ = getParam("batch_size", 300);
    std_v_ = getParam("std_v", 0.1);
    std_w_ = getParam("std_w", 0.3);
    limit_v_ = getParam("limit_v", 0.5);
    limit_w_ = getParam("limit_w", 1.0);
    iteration_count_ = getParam("iteration_count", 1);
    temperature_ = getParam("temperature", 0.25);
  }

  void resetBatches() {
    batches_ = xt::zeros<float>({batch_size_, time_steps_, last_dim_});
    control_sequence_ = xt::zeros<float>({time_steps_, control_dim_size_});
    xt::view(batches_, xt::all(), xt::all(), 4) = model_dt_;
  }

  Tensor
  generateNoisedTrajectoryBatches(const geometry_msgs::msg::Twist &twist) {
    getControlBatches() = generateNoisedControlBatches();
    applyControlConstraints();
    setBatchesVelocity(twist);
    return integrateVelocityBatches();
  }

  auto generateNoisedControlBatches() -> Tensor {
    auto v_noises =
        xt::random::randn<T>({batch_size_, time_steps_, 1}, 0.0, std_v_);
    auto w_noises =
        xt::random::randn<T>({batch_size_, time_steps_, 1}, 0.0, std_w_);
    return control_sequence_ +
           xt::concatenate(xt::xtuple(v_noises, w_noises), 2);
  }

  void applyControlConstraints() {
    auto v = getLinearVelocityControlBatches();
    auto w = getAngularVelocityControlBatches();

    v = xt::clip(v, -limit_v_, limit_v_);
    w = xt::clip(w, -limit_w_, limit_w_);
  }

  void setBatchesVelocity(const geometry_msgs::msg::Twist &twist) {
    setBatchesInitialVelocities(twist);
    propagateBatchesVelocityFromInitials();
  }

  void setBatchesInitialVelocities(const geometry_msgs::msg::Twist &twist) {
    xt::view(batches_, xt::all(), 0, 0) = twist.linear.x;
    xt::view(batches_, xt::all(), 0, 1) = twist.angular.z;
  }

  void propagateBatchesVelocityFromInitials() {
    using namespace xt::placeholders;

    for (int t = 0; t < time_steps_ - 1; t++) {
      auto curr_batch = xt::view(batches_, xt::all(), t); // -> batch x 5
      auto next_batch_velocities =
          xt::view(batches_, xt::all(), t + 1, xt::range(_, 2)); // batch x 2

      next_batch_velocities = model_(curr_batch);
    }
  }

  auto integrateVelocityBatches() const -> Tensor {
    using namespace xt::placeholders;

    auto v = xt::view(batches_, xt::all(), xt::all(), 0);
    auto w = xt::view(batches_, xt::all(), xt::all(), 1);
    auto yaw = xt::cumsum(w * model_dt_, 1);
    yaw -= xt::view(yaw, xt::all(), xt::range(_, 1));

    auto x = xt::cumsum(v * model_dt_ * xt::cos(yaw), 1);
    auto y = xt::cumsum(w * model_dt_ * xt::sin(yaw), 1);

    x -= xt::view(x, xt::all(), xt::range(_, 1));
    y -= xt::view(x, xt::all(), xt::range(_, 1));

    return xt::concatenate(
        xt::xtuple(xt::view(x, xt::all(), xt::all(), xt::newaxis()),
                   xt::view(y, xt::all(), xt::all(), xt::newaxis()),
                   xt::view(yaw, xt::all(), xt::all(), xt::newaxis())),
        2);
  }

  Tensor evalBatchesCosts(const Tensor &trajectory_batches,
                          const nav_msgs::msg::Path &path) const {

    std::vector<size_t> shape = {trajectory_batches.shape()[0]};

    auto reference_cost = xt::zeros<T>(shape);
    auto obstacle_cost = xt::zeros<T>(shape);

    auto goal_cost = [&](int weight, int power) {
      Tensor last_goal = {static_cast<T>(path.poses.back().pose.position.x),
                          static_cast<T>(path.poses.back().pose.position.y)};

      Tensor x = xt::view(trajectory_batches, xt::all(), xt::all(), 0);
      Tensor y = xt::view(trajectory_batches, xt::all(), xt::all(), 1);

      auto dx = x - last_goal[0];
      auto dy = y - last_goal[1];

      auto dists = xt::hypot(dx, dy);

      auto result = weight * xt::pow(xt::view(dists, xt::all(), -1), power);
      return Tensor(result);
    };

    return obstacle_cost + reference_cost + goal_cost(2, 2);
  }

  void updateControlSequence(Tensor &costs) {
    costs -= xt::amin(costs, xt::evaluation_strategy::immediate);
    Tensor exponents = xt::exp(-1 / temperature_ * costs);
    auto softmaxes =
        exponents / xt::sum(exponents, xt::evaluation_strategy::immediate);

    Tensor softmaxes_expanded =
        xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

    control_sequence_ = xt::sum(getControlBatches() * softmaxes_expanded, 0);
  }

  template <typename H>
  geometry_msgs::msg::TwistStamped getControlFromSequence(const H &header) {
    return geometry::toTwistStamped(xt::view(control_sequence_, 0),
                                           header);
  }

  decltype(auto) getControlBatches() {
    return xt::view(batches_, xt::all(), xt::all(), xt::range(2, 4));
  }

  decltype(auto) getLinearVelocityControlBatches() {
    return xt::view(batches_, xt::all(), xt::all(), 2);
  }

  decltype(auto) getAngularVelocityControlBatches() {
    return xt::view(batches_, xt::all(), xt::all(), 3);
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::string node_name_;
  nav2_costmap_2d::Costmap2D *costmap_;

  static constexpr int last_dim_ = 5;
  static constexpr int control_dim_size_ = 2;

  int batch_size_;
  int time_steps_;
  int iteration_count_;

  double model_dt_;
  double std_v_;
  double std_w_;
  double limit_v_;
  double limit_w_;
  double temperature_;

  Tensor batches_;
  Tensor control_sequence_;
  Tensor trajectories_;

  std::function<Model> model_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPI Optimizer")};
};

} // namespace mppi::optimization
