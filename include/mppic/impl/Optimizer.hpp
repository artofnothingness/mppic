#pragma once

#include <limits>
#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>

#include "mppic/Optimizer.hpp"
#include "mppic/utils/LineIterator.hpp"
#include "mppic/utils/common.hpp"
#include "mppic/utils/geometry.hpp"

namespace mppi::optimization {

template <typename T>
geometry_msgs::msg::TwistStamped
Optimizer<T>::evalNextBestControl(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &robot_speed,
    const nav_msgs::msg::Path &plan) {
  for (size_t i = 0; i < iteration_count_; ++i) {
    generated_trajectories_ = generateNoisedTrajectories(robot_pose, robot_speed);
    auto costs =
        critic_scorer_.evalTrajectoriesScores(generated_trajectories_, plan, robot_pose);
    updateControlSequence(costs);
  }

  return geometry::toTwistStamped(getControlFromSequence(0), plan.header.stamp,
                                  costmap_ros_->getBaseFrameID());
}

template <typename T>
void
Optimizer<T>::on_configure(rclcpp_lifecycle::LifecycleNode *const parent,
                           const std::string &node_name,
                           nav2_costmap_2d::Costmap2DROS *const costmap_ros,
                           model_t &&model) {
  parent_ = parent;
  node_name_ = node_name;
  costmap_ros_ = costmap_ros;
  model_ = model;
  costmap_ = costmap_ros_->getCostmap();

  getParams();
  configureComponents();
  reset();
  RCLCPP_INFO(logger_, "Configured");
}

template <typename T>
void
Optimizer<T>::getParams() {
  auto setParam = utils::getParamSetter(parent_, node_name_);

  setParam(model_dt_, "model_dt", 0.1);
  setParam(time_steps_, "time_steps", 15);
  setParam(batch_size_, "batch_size", 200);
  setParam(v_std_, "v_std", 0.1);
  setParam(w_std_, "w_std", 0.3);
  setParam(v_limit_, "v_limit", 0.5);
  setParam(w_limit_, "w_limit", 1.3);
  setParam(iteration_count_, "iteration_count", 2);
  setParam(temperature_, "temperature", 0.25);
  setParam(approx_reference_cost_, "approx_reference_cost", false);
}

template <typename T>
void
Optimizer<T>::configureComponents() {
  // TODO pluginize

  std::vector<std::unique_ptr<optimization::CriticFunction<T>>> critics;

  critics.push_back(std::make_unique<optimization::GoalCritic<T>>());
  critics.push_back(std::make_unique<optimization::GoalAngleCritic<T>>());
  critics.push_back(std::make_unique<optimization::ObstaclesCritic<T>>());

  if (approx_reference_cost_) {
    critics.push_back(
        std::make_unique<optimization::approxReferenceTrajectoryCritic<T>>());
  } else {
    critics.push_back(
        std::make_unique<optimization::referenceTrajectoryCritic<T>>());
  }


  critic_scorer_ = optimization::CriticScorer<T>(std::move(critics));
  critic_scorer_.on_configure(parent_, node_name_, costmap_ros_);
}

template <typename T>
void
Optimizer<T>::reset() {
  state_.data =
      xt::zeros<T>({batch_size_, time_steps_, batches_last_dim_size_});
  state_.getTimeIntervals() = model_dt_;
  control_sequence_.reset({time_steps_, control_dim_size_});
}

template <typename T>
xt::xtensor<T, 3>
Optimizer<T>::generateNoisedTrajectories(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &robot_speed) {
  state_.getControls() = generateNoisedControlBatches();
  applyControlConstraints();
  evalBatchesVelocities(state_, robot_speed);
  return integrateBatchesVelocities(state_, robot_pose);
}

template <typename T>
xt::xtensor<T, 3>
Optimizer<T>::generateNoisedControlBatches() const {
  auto v_noises =
      xt::random::randn<T>({batch_size_, time_steps_, 1U}, 0.0, v_std_);
  auto w_noises =
      xt::random::randn<T>({batch_size_, time_steps_, 1U}, 0.0, w_std_);
  return control_sequence_.data +
         xt::concatenate(xt::xtuple(v_noises, w_noises), 2);
}

template <typename T>
void
Optimizer<T>::applyControlConstraints() {
  auto v = state_.getControlLinearVelocities();
  auto w = state_.getControlAngularVelocities();

  v = xt::clip(v, -v_limit_, v_limit_);
  w = xt::clip(w, -w_limit_, w_limit_);
}

template <typename T>
void
Optimizer<T>::evalBatchesVelocities(
    auto &state, const geometry_msgs::msg::Twist &robot_speed) const {
  setBatchesInitialVelocities(state, robot_speed);
  propagateBatchesVelocitiesFromInitials(state);
}

template <typename T>
void
Optimizer<T>::setBatchesInitialVelocities(
    auto &state, const geometry_msgs::msg::Twist &robot_speed) const {
  xt::view(state.getLinearVelocities(), xt::all(), 0) = robot_speed.linear.x;
  xt::view(state.getAngularVelocities(), xt::all(), 0) = robot_speed.angular.z;
}

template <typename T>
void
Optimizer<T>::propagateBatchesVelocitiesFromInitials(auto &state) const {
  using namespace xt::placeholders;

  for (size_t t = 0; t < time_steps_ - 1; t++) {
    auto curr_state = xt::view(state.data, xt::all(), t);  // -> batch x 5
    auto next_vels =
        xt::view(state.getVelocities(), xt::all(), t + 1);  // batch x 2

    next_vels = model_(curr_state);
  }
}

template <typename T>
xt::xtensor<T, 2>
Optimizer<T>::evalTrajectoryFromControlSequence(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &robot_speed) const {
  State<T> state;
  state.data = xt::zeros<T>({1U, time_steps_, batches_last_dim_size_});
  state.getControls() = control_sequence_.data;
  state.getTimeIntervals() = model_dt_;

  evalBatchesVelocities(state, robot_speed);
  return xt::squeeze(integrateBatchesVelocities(state, robot_pose));
}

template <typename T>
xt::xtensor<T, 3>
Optimizer<T>::integrateBatchesVelocities(
    const auto &state, const geometry_msgs::msg::PoseStamped &pose) const {
  using namespace xt::placeholders;

  auto v = state.getLinearVelocities();
  auto w = state.getAngularVelocities();
  auto yaw = xt::cumsum(w * model_dt_, 1);

  auto yaw_offseted = yaw;
  xt::view(yaw_offseted, xt::all(), xt::range(1, _)) =
      xt::eval(xt::view(yaw, xt::all(), xt::range(_, -1)));
  xt::view(yaw_offseted, xt::all(), 0) = 0;
  xt::view(yaw_offseted, xt::all(), xt::all()) +=
      tf2::getYaw(pose.pose.orientation);

  auto v_x = v * xt::cos(yaw_offseted);
  auto v_y = v * xt::sin(yaw_offseted);

  auto x = pose.pose.position.x + xt::cumsum(v_x * model_dt_, 1);
  auto y = pose.pose.position.y + xt::cumsum(v_y * model_dt_, 1);

  return xt::concatenate(
      xt::xtuple(xt::view(x, xt::all(), xt::all(), xt::newaxis()),
                 xt::view(y, xt::all(), xt::all(), xt::newaxis()),
                 xt::view(yaw, xt::all(), xt::all(), xt::newaxis())),
      2);
}

template <typename T>
void
Optimizer<T>::updateControlSequence(const xt::xtensor<T, 1> &costs) {
  using xt::evaluation_strategy::immediate;

  auto &&costs_normalized = costs - xt::amin(costs, immediate);
  auto exponents = xt::eval(xt::exp(-1 / temperature_ * costs_normalized));
  auto softmaxes = exponents / xt::sum(exponents, immediate);
  auto softmaxes_expanded =
      xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

  control_sequence_.data =
      xt::sum(state_.getControls() * softmaxes_expanded, 0);
}

template <typename T>
auto
Optimizer<T>::getControlFromSequence(unsigned int offset) {
  return xt::view(control_sequence_.data, offset);
}

}  // namespace mppi::optimization
