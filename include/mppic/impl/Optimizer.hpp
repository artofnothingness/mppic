#pragma once

#include <limits>

#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>

#include "mppic/MotionModel.hpp"
#include "mppic/Optimizer.hpp"
#include "mppic/utils/LineIterator.hpp"
#include "mppic/utils/common.hpp"
#include "mppic/utils/geometry.hpp"

namespace mppi::optimization {
template<typename T>
geometry_msgs::msg::TwistStamped Optimizer<T>::evalControl(
  const geometry_msgs::msg::PoseStamped &robot_pose,
  const geometry_msgs::msg::Twist &robot_speed,
  const nav_msgs::msg::Path &plan)
{
  for (size_t i = 0; i < iteration_count_; ++i) {
    generated_trajectories_ = generateNoisedTrajectories(robot_pose, robot_speed);
    auto costs = critic_scorer_.evalTrajectoriesScores(generated_trajectories_, plan, robot_pose);
    updateControlSequence(costs);
  }

  return geometry::toTwistStamped(
    getControlFromSequence(0), plan.header.stamp, costmap_ros_->getBaseFrameID());
}

template<typename T>
void Optimizer<T>::on_configure(rclcpp_lifecycle::LifecycleNode *const parent,
  const std::string &node_name,
  nav2_costmap_2d::Costmap2DROS *const costmap_ros,
  model_t model)
{
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

template<typename T>
void Optimizer<T>::getParams()
{
  auto getParam = utils::getParamGetter(parent_, node_name_);

  getParam(model_dt_, "model_dt", 0.1);
  getParam(time_steps_, "time_steps", 15);
  getParam(batch_size_, "batch_size", 200);
  getParam(iteration_count_, "iteration_count", 2);
  getParam(temperature_, "temperature", 0.25);
  getParam(approx_reference_cost_, "approx_reference_cost", false);

  getParam(vx_max_, "vx_max", 0.5);
  getParam(vy_max_, "vy_max", 1.3);
  getParam(wz_max_, "wz_max", 1.3);
  getParam(vx_std_, "vx_std", 0.1);
  getParam(vy_std_, "vy_std", 0.1);
  getParam(wz_std_, "wz_std", 0.3);

  std::string name;
  getParam(name, "motion_model", std::string("diff"));

  auto &nmap = motion_model_name_map_;
  auto &cmap = motion_model_control_dim_map_;

  if (auto it = nmap.find(name); it != nmap.end()) {
    setMotionModel(it->second);
  } else {
    RCLCPP_INFO(logger_, "Motion model is unknown, use default/previous");
  }

  if (auto it = cmap.find(getMotionModel()); it != cmap.end()) { control_vec_dim_ = it->second; }
}

// TODO pluginize
template<typename T>
void Optimizer<T>::configureComponents()
{
  std::vector<std::unique_ptr<optimization::CriticFunction<T>>> critics;

  critics.push_back(std::make_unique<optimization::GoalCritic<T>>());
  critics.push_back(std::make_unique<optimization::GoalAngleCritic<T>>());
  critics.push_back(std::make_unique<optimization::ObstaclesCritic<T>>());

  if (approx_reference_cost_) {
    critics.push_back(std::make_unique<optimization::approxReferenceTrajectoryCritic<T>>());
  } else {
    critics.push_back(std::make_unique<optimization::referenceTrajectoryCritic<T>>());
  }

  critic_scorer_ = optimization::CriticScorer<T>(std::move(critics));

  std::string component_name = "CriticScorer";
  critic_scorer_.on_configure(parent_, node_name_, component_name, costmap_ros_);
}

// Imple dependce on Y
template<typename T>
void Optimizer<T>::reset()
{
  state_.reset(batch_size_, time_steps_);
  state_.getTimeIntervals() = model_dt_;
  control_sequence_ = xt::zeros<T>({ time_steps_, control_vec_dim_ });
}

// Imple dependce on Y
template<typename T>
xt::xtensor<T, 3> Optimizer<T>::generateNoisedTrajectories(
  const geometry_msgs::msg::PoseStamped &robot_pose,
  const geometry_msgs::msg::Twist &robot_speed)
{
  state_.getControls() = generateNoisedControls();
  applyControlConstraints();
  updateStateVelocities(state_, robot_speed);
  return integrateStateVelocities(state_, robot_pose);
}

template<typename T>
xt::xtensor<T, 3> Optimizer<T>::generateNoisedControls() const
{
  auto vx_noises = xt::random::randn<T>({ batch_size_, time_steps_, 1U }, 0.0, vx_std_);
  auto wz_noises = xt::random::randn<T>({ batch_size_, time_steps_, 1U }, 0.0, wz_std_);

  if (isHolonomic(getMotionModel())) {
    auto vy_noises = xt::random::randn<T>({ batch_size_, time_steps_, 1U }, 0.0, vy_std_);
    return control_sequence_ + xt::concatenate(xt::xtuple(vx_noises, wz_noises, vy_noises), 2);
  }

  return control_sequence_ + xt::concatenate(xt::xtuple(vx_noises, wz_noises), 2);
}

// Imple dependce on Y
template<typename T>
void Optimizer<T>::applyControlConstraints()
{
  auto vx = state_.getControlVelocitiesVX();
  auto wz = state_.getControlVelocitiesWZ();

  if (isHolonomic(getMotionModel())) {
    auto vy = state_.getControlVelocitiesVY();
    vy = xt::clip(vy, -vy_max_, vy_max_);
  }

  vx = xt::clip(vx, -vx_max_, vx_max_);
  wz = xt::clip(wz, -wz_max_, wz_max_);
}

template<typename T>
void Optimizer<T>::updateStateVelocities(auto &state,
  const geometry_msgs::msg::Twist &robot_speed) const
{
  updateInitialStateVelocities(state, robot_speed);
  propagateStateVelocitiesFromInitials(state);
}

template<typename T>
void Optimizer<T>::updateInitialStateVelocities(auto &state,
  const geometry_msgs::msg::Twist &robot_speed) const
{
  xt::view(state.getVelocitiesVX(), xt::all(), 0) = robot_speed.linear.x;
  xt::view(state.getVelocitiesWZ(), xt::all(), 0) = robot_speed.angular.z;

  if (isHolonomic(getMotionModel())) {
    xt::view(state.getVelocitiesVY(), xt::all(), 0) = robot_speed.linear.y;
  }
}

// Imple dependce on Y
template<typename T>
void Optimizer<T>::propagateStateVelocitiesFromInitials(auto &state) const
{
  using namespace xt::placeholders;

  for (size_t t = 0; t < time_steps_ - 1; t++) {
    auto curr_state = xt::view(state.data, xt::all(), t);
    auto next_vels = xt::view(state.getVelocities(), xt::all(), t + 1);

    next_vels = model_(curr_state);
  }
}

// Imple dependce on Y
template<typename T>
xt::xtensor<T, 2> Optimizer<T>::evalTrajectoryFromControlSequence(
  const geometry_msgs::msg::PoseStamped &robot_pose,
  const geometry_msgs::msg::Twist &robot_speed) const
{
  State<T> state;
  state.setMotionModel(getMotionModel());
  state.reset(1U, time_steps_);
  state.getControls() = control_sequence_;
  state.getTimeIntervals() = model_dt_;

  updateStateVelocities(state, robot_speed);
  return xt::squeeze(integrateStateVelocities(state, robot_pose));
}

// Imple dependce on Y
template<typename T>
xt::xtensor<T, 3> Optimizer<T>::integrateStateVelocities(const auto &state,
  const geometry_msgs::msg::PoseStamped &pose) const
{
  using namespace xt::placeholders;

  auto v = state.getVelocitiesVX();
  auto w = state.getVelocitiesWZ();
  auto yaw = xt::cumsum(w * model_dt_, 1);

  auto yaw_offseted = yaw;
  xt::view(yaw_offseted, xt::all(), xt::range(1, _)) =
    xt::eval(xt::view(yaw, xt::all(), xt::range(_, -1)));
  xt::view(yaw_offseted, xt::all(), 0) = 0;
  xt::view(yaw_offseted, xt::all(), xt::all()) += tf2::getYaw(pose.pose.orientation);

  auto v_x = v * xt::cos(yaw_offseted);
  auto v_y = v * xt::sin(yaw_offseted);

  auto x = pose.pose.position.x + xt::cumsum(v_x * model_dt_, 1);
  auto y = pose.pose.position.y + xt::cumsum(v_y * model_dt_, 1);

  return xt::concatenate(xt::xtuple(xt::view(x, xt::all(), xt::all(), xt::newaxis()),
                           xt::view(y, xt::all(), xt::all(), xt::newaxis()),
                           xt::view(yaw, xt::all(), xt::all(), xt::newaxis())),
    2);
}

template<typename T>
void Optimizer<T>::updateControlSequence(const xt::xtensor<T, 1> &costs)
{
  using xt::evaluation_strategy::immediate;

  auto &&costs_normalized = costs - xt::amin(costs, immediate);
  auto exponents = xt::eval(xt::exp(-1 / temperature_ * costs_normalized));
  auto softmaxes = exponents / xt::sum(exponents, immediate);
  auto softmaxes_expanded = xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

  control_sequence_ = xt::sum(state_.getControls() * softmaxes_expanded, 0);
}

template<typename T>
auto Optimizer<T>::getControlFromSequence(unsigned int offset)
{
  return xt::view(control_sequence_, offset);
}

template<typename T>
MotionModel Optimizer<T>::getMotionModel() const
{
  return motion_model_t_;
}

template<typename T>
void Optimizer<T>::setMotionModel(MotionModel motion_model)
{
  motion_model_t_ = motion_model;
  state_.setMotionModel(motion_model);
}

}// namespace mppi::optimization
