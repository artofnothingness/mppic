#pragma once

#include "mppi/Optimizer.hpp"

#include "xtensor/xmath.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor/xslice.hpp"

#include "xtensor/xio.hpp"

#include "utils/common.hpp"
#include "utils/geometry.hpp"

namespace mppi::optimization {

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::evalNextControl(
    const geometry_msgs::msg::Twist &twist, const nav_msgs::msg::Path &path)
    -> geometry_msgs::msg::TwistStamped {

  static Tensor costs;
  for (int i = 0; i < iteration_count_; ++i) {
    generated_trajectories_ = generateNoisedTrajectories(twist);
    costs = evalBatchesCosts(generated_trajectories_, path);
    updateControlSequence(costs);
  }

  return getControlFromSequence(path.header);
}

template <typename T, typename Tensor, typename Model>
void Optimizer<T, Tensor, Model>::on_configure() {
  getParams();
  resetBatches();
  RCLCPP_INFO(logger_, "Configured");
}

template <typename T, typename Tensor, typename Model>
void Optimizer<T, Tensor, Model>::getParams() {

  auto getParam = [&](const std::string &param_name, auto default_value) {
    std::string name = node_name_ + '.' + param_name;
    return utils::getParam(name, default_value, parent_);
  };

  model_dt_ = getParam("model_dt", 0.1);
  time_steps_ = getParam("time_steps", 30);
  batch_size_ = getParam("batch_size", 300);
  v_std_ = getParam("v_std", 0.1);
  w_std_ = getParam("w_std", 0.5);
  v_limit_ = getParam("v_limit", 0.5);
  w_limit_ = getParam("w_limit", 1.3);
  iteration_count_ = getParam("iteration_count", 1);
  temperature_ = getParam("temperature", 0.25);
}

template <typename T, typename Tensor, typename Model>
void Optimizer<T, Tensor, Model>::resetBatches() {
  batches_ = xt::zeros<T>({batch_size_, time_steps_, last_dim_size});
  control_sequence_ = xt::zeros<T>({time_steps_, control_dim_size_});
  xt::view(batches_, xt::all(), xt::all(), 4) = model_dt_;
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::generateNoisedTrajectories(
    const geometry_msgs::msg::Twist &twist) -> Tensor {
  getBatchesControls() = generateNoisedControlBatches();
  applyControlConstraints();
  setBatchesVelocities(twist);
  return integrateBatchesVelocities();
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::generateNoisedControlBatches() -> Tensor {
  auto v_noises =
      xt::random::randn<T>({batch_size_, time_steps_, 1}, 0.0, v_std_);
  auto w_noises =
      xt::random::randn<T>({batch_size_, time_steps_, 1}, 0.0, w_std_);
  return control_sequence_ + xt::concatenate(xt::xtuple(v_noises, w_noises), 2);
}

template <typename T, typename Tensor, typename Model>
void Optimizer<T, Tensor, Model>::applyControlConstraints() {
  auto v = getBatchesControlLinearVelocities();
  auto w = getBatchesControlAngularVelocities();

  v = xt::clip(v, -v_limit_, v_limit_);
  w = xt::clip(w, -w_limit_, w_limit_);
}

template <typename T, typename Tensor, typename Model>
void Optimizer<T, Tensor, Model>::setBatchesVelocities(
    const geometry_msgs::msg::Twist &twist) {
  setBatchesInitialVelocities(twist);
  propagateBatchesVelocitiesFromInitials();
}

template <typename T, typename Tensor, typename Model>
void Optimizer<T, Tensor, Model>::setBatchesInitialVelocities(
    const geometry_msgs::msg::Twist &twist) {
  xt::view(batches_, xt::all(), 0, 0) = twist.linear.x;
  xt::view(batches_, xt::all(), 0, 1) = twist.angular.z;
}

template <typename T, typename Tensor, typename Model>
void Optimizer<T, Tensor, Model>::propagateBatchesVelocitiesFromInitials() {
  using namespace xt::placeholders;

  for (int t = 0; t < time_steps_ - 1; t++) {
    auto curr_batch = xt::view(batches_, xt::all(), t); // -> batch x 5
    auto next_batch_velocities =
        xt::view(batches_, xt::all(), t + 1, xt::range(_, 2)); // batch x 2
    next_batch_velocities = model_(curr_batch);
  }
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::integrateBatchesVelocities() const -> Tensor {
  using namespace xt::placeholders;

  auto v = getBatchesLinearVelocities();
  auto w = getBatchesAngularVelocities();

  auto yaw = xt::cumsum(w * model_dt_, 1);

  xt::view(yaw, xt::all(), xt::range(1, _)) =
      xt::view(yaw, xt::all(), xt::range(_, -1));
  xt::view(yaw, xt::all(), 0) = 0;

  auto v_x = v * xt::cos(yaw);
  auto v_y = v * xt::sin(yaw);

  auto x = xt::cumsum(v_x * model_dt_, 1);
  auto y = xt::cumsum(v_y * model_dt_, 1);

  return xt::concatenate(
      xt::xtuple(xt::view(x, xt::all(), xt::all(), xt::newaxis()),
                 xt::view(y, xt::all(), xt::all(), xt::newaxis()),
                 xt::view(yaw, xt::all(), xt::all(), xt::newaxis())),
      2);
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::evalBatchesCosts(
    const Tensor &trajectory_batches, const nav_msgs::msg::Path &path) const
    -> Tensor {

  using namespace xt::placeholders;

  std::vector<size_t> shape = {trajectory_batches.shape()[0]};

  auto obstacle_cost = xt::zeros<T>(shape);

  /* auto reference_cost = [&](int weight, int power) -> Tensor { */
    /* (void)weight; */
  /*   (void)power; */

  /*   if (path.poses.empty()) */
  /*     return xt::zeros<T>(shape); */

  /*   auto cost = xt::mean(dists_to_segments, {0, 2}); */

  /*   return xt::zeros<T>(shape); */
  /*   weight xt::pow(cost, power); */
  /* }; */

  auto goal_cost = [&](int weight, int power) -> Tensor {
    if (path.poses.empty())
      return xt::zeros<T>(shape);

    Tensor last_goal = {static_cast<T>(path.poses.back().pose.position.x),
                        static_cast<T>(path.poses.back().pose.position.y)};

    Tensor x = xt::view(trajectory_batches, xt::all(), xt::all(), 0);
    Tensor y = xt::view(trajectory_batches, xt::all(), xt::all(), 1);

    auto dx = x - last_goal(0);
    auto dy = y - last_goal(1);

    auto dists = xt::hypot(dx, dy);

    auto cost = xt::view(dists, xt::all(), -1);

    return weight * xt::pow(cost, power);
  };

  return goal_cost(2, 2);
}

template <typename T, typename Tensor, typename Model>
void Optimizer<T, Tensor, Model>::updateControlSequence(const Tensor &costs) {
  Tensor costs_normalized =
      costs - xt::amin(costs, xt::evaluation_strategy::immediate);
  Tensor exponents = xt::exp(-1 / temperature_ * costs);
  auto softmaxes =
      exponents / xt::sum(exponents, xt::evaluation_strategy::immediate);

  Tensor softmaxes_expanded =
      xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

  control_sequence_ = xt::sum(getBatchesControls() * softmaxes_expanded, 0);
}

template <typename T, typename Tensor, typename Model>
template <typename H>
auto Optimizer<T, Tensor, Model>::getControlFromSequence(const H &header)
    -> geometry_msgs::msg::TwistStamped {

  return geometry::toTwistStamped(xt::view(control_sequence_, 0), header);
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::getBatchesLinearVelocities() const {
  return xt::view(batches_, xt::all(), xt::all(), 0);
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::getBatchesAngularVelocities() const {
  return xt::view(batches_, xt::all(), xt::all(), 1);
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::getBatchesControlLinearVelocities() const {
  return xt::view(batches_, xt::all(), xt::all(), 2);
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::getBatchesControlAngularVelocities() const {
  return xt::view(batches_, xt::all(), xt::all(), 3);
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::getBatchesControls() const {
  return xt::view(batches_, xt::all(), xt::all(), xt::range(2, 4));
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::getBatchesLinearVelocities() {
  return xt::view(batches_, xt::all(), xt::all(), 0);
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::getBatchesAngularVelocities() {
  return xt::view(batches_, xt::all(), xt::all(), 1);
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::getBatchesControls() {
  return xt::view(batches_, xt::all(), xt::all(), xt::range(2, 4));
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::getBatchesControlLinearVelocities() {
  return xt::view(batches_, xt::all(), xt::all(), 2);
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::getBatchesControlAngularVelocities() {
  return xt::view(batches_, xt::all(), xt::all(), 3);
}

} // namespace mppi::optimization
