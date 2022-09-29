// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/optimizer.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace mppi
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void Optimizer::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  ParametersHandler * param_handler)
{
  parent_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  parameters_handler_ = param_handler;

  auto node = parent_.lock();
  logger_ = node->get_logger();

  getParams();

  critic_manager_.on_configure(parent_, name_, costmap_ros_, parameters_handler_);
  noise_generator_.initialize(settings_, isHolonomic());

  reset();
}

void Optimizer::shutdown()
{
  noise_generator_.shutdown();
}

void Optimizer::getParams()
{
  std::string motion_model_name;

  auto & s = settings_;
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter("");
  getParam(s.model_dt, "model_dt", 0.1f);
  getParam(s.time_steps, "time_steps", 15);
  getParam(s.batch_size, "batch_size", 400);
  getParam(s.iteration_count, "iteration_count", 1);
  getParam(s.temperature, "temperature", 0.25f);
  getParam(s.base_constraints.vx_max, "vx_max", 0.5);
  getParam(s.base_constraints.vx_min, "vx_min", -0.35);
  getParam(s.base_constraints.vy, "vy_max", 0.5);
  getParam(s.base_constraints.wz, "wz_max", 1.3);
  getParam(s.sampling_std.vx, "vx_std", 0.2);
  getParam(s.sampling_std.vy, "vy_std", 0.2);
  getParam(s.sampling_std.wz, "wz_std", 1.0);
  getParam(s.retry_attempt_limit, "retry_attempt_limit", 1);

  getParam(motion_model_name, "motion_model", std::string("DiffDrive"));

  s.constraints = s.base_constraints;
  setMotionModel(motion_model_name);
  parameters_handler_->addPostCallback([this]() {reset();});

  double controller_frequency;
  getParentParam(controller_frequency, "controller_frequency", 0.0, ParameterType::Static);
  setOffset(controller_frequency);
}

void Optimizer::setOffset(double controller_frequency)
{
  const double controller_period = 1.0 / controller_frequency;
  constexpr double eps = 1e-6;

  if (controller_period < settings_.model_dt) {
    RCLCPP_WARN(
      logger_,
      "Controller period is less then model dt, consider setting it equal");
  } else if (abs(controller_period - settings_.model_dt) < eps) {
    RCLCPP_INFO(
      logger_,
      "Controller period is equal to model dt. Control seuqence "
      "shifting is ON");
    settings_.shift_control_sequence = true;
  } else {
    throw std::runtime_error(
            "Controller period more then model dt, set it equal to model dt");
  }
}

void Optimizer::reset()
{
  state_.reset(settings_.batch_size, settings_.time_steps);
  control_sequence_.reset(settings_.time_steps);
  action_sequence_.reset(settings_.time_steps);

  costs_ = xt::zeros<float>({settings_.batch_size});
  generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

  noise_generator_.reset(settings_, isHolonomic());
  RCLCPP_INFO(logger_, "Optimizer reset");
}

geometry_msgs::msg::TwistStamped Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  prepare(robot_pose, robot_speed, plan, goal_checker);

  do {
    optimize();
  } while (fallback(critics_data_.fail_flag));

  auto control = getActionFromSequenceAsTwist(plan.header.stamp);

  if (settings_.shift_control_sequence) {
    shiftControlSequence();
    shiftActionSequence();
  }

  return control;
}

void Optimizer::optimize()
{
  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generateNoisedTrajectories();
    critic_manager_.evalTrajectoriesScores(critics_data_);
    updateControlSequence();
    updateActionSequence();
  }
}

bool Optimizer::fallback(bool fail)
{
  static size_t counter = 0;

  if (!fail) {
    counter = 0;
    return false;
  }

  reset();

  if (counter++ > settings_.retry_attempt_limit) {
    counter = 0;
    throw std::runtime_error("Optimizer fail to compute path");
  }

  return true;
}

void Optimizer::prepare(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  state_.pose = robot_pose;
  state_.speed = robot_speed;
  path_ = utils::toTensor(plan);
  costs_.fill(0);

  critics_data_.fail_flag = false;
  critics_data_.goal_checker = goal_checker;
}

void Optimizer::shiftControlSequence()
{
  using namespace xt::placeholders;  // NOLINT
  control_sequence_.vx = xt::roll(control_sequence_.vx, -1);
  control_sequence_.wz = xt::roll(control_sequence_.wz, -1);

  xt::view(control_sequence_.vx, -1) =
    xt::view(control_sequence_.vx, -2);

  xt::view(control_sequence_.wz, -1) =
    xt::view(control_sequence_.wz, -2);

  if (isHolonomic()) {
    control_sequence_.vy = xt::roll(control_sequence_.vy, -1);
    xt::view(control_sequence_.vy, -1) =
      xt::view(control_sequence_.vy, -2);
  }
}

void Optimizer::shiftActionSequence()
{
  using namespace xt::placeholders;  // NOLINT
  action_sequence_.vx = xt::roll(action_sequence_.vx, -1);
  action_sequence_.wz = xt::roll(action_sequence_.wz, -1);

  xt::view(action_sequence_.vx, -1) =
    xt::view(action_sequence_.vx, -2);

  xt::view(action_sequence_.wz, -1) =
    xt::view(action_sequence_.wz, -2);

  if (isHolonomic()) {
    action_sequence_.vy = xt::roll(action_sequence_.vy, -1);
    xt::view(action_sequence_.vy, -1) =
      xt::view(action_sequence_.vy, -2);
  }
}

void Optimizer::generateNoisedTrajectories()
{
  noise_generator_.setNoisedControls(state_, control_sequence_);
  noise_generator_.generateNextNoises();
  generateActionSequence();
  applyVelocityConstraints();
  updateStateVelocities(state_);
  integrateStateVelocities(generated_trajectories_, state_);
}

bool Optimizer::isHolonomic() const {return motion_model_->isHolonomic();}

void Optimizer::applyVelocityConstraints()
{
  auto & s = settings_;

  // TODO should still apply clipped limits to controls?
  if (isHolonomic()) {
    state_.avy = xt::clip(state_.avy, -s.constraints.vy, s.constraints.vy);
    state_.cvy = xt::clip(state_.cvy, -s.constraints.vy, s.constraints.vy);
  }

  state_.cvx = xt::clip(state_.cvx, s.constraints.vx_min, s.constraints.vx_max);
  state_.cwz = xt::clip(state_.cwz, -s.constraints.wz, s.constraints.wz);

  motion_model_->applyConstraints(state_); // TODO this needs to be called after predict to have vx/vw populated, or do cvx/avx
}

void Optimizer::generateActionSequence()
{
  // TODO action sequence tensor for batches: a_t^k = a_t + v_t^k + delta t
  // should be state_.cvx or state_.vx? Control seems to work better,
  // but doesn't seem technically right.
  state_.avx = action_sequence_.vx + (state_.cvx * settings_.model_dt);
  state_.awz = action_sequence_.wz + (state_.cwz * settings_.model_dt);

  if (isHolonomic()) {
    state_.avy = action_sequence_.vy + (state_.cvy * settings_.model_dt);
  }
}

void Optimizer::updateStateVelocities(
  models::State & state) const
{
  updateInitialStateVelocities(state);
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(
  models::State & state) const
{
  xt::noalias(xt::view(state.vx, xt::all(), 0)) = state_.speed.linear.x;
  xt::noalias(xt::view(state.wz, xt::all(), 0)) = state_.speed.angular.z;

  if (isHolonomic()) {
    xt::noalias(xt::view(state.vy, xt::all(), 0)) = state_.speed.linear.y;
  }
}

void Optimizer::propagateStateVelocitiesFromInitials(
  models::State & state) const
{
  motion_model_->predict(state);
}

void Optimizer::integrateStateVelocities(
  xt::xtensor<float, 2> & trajectory,
  const xt::xtensor<float, 2> & state) const
{
  double initial_yaw = tf2::getYaw(state_.pose.pose.orientation);

  const auto vx = xt::view(state, xt::all(), 0);
  const auto vy = xt::view(state, xt::all(), 2);
  const auto wz = xt::view(state, xt::all(), 1);

  auto traj_x = xt::view(trajectory, xt::all(), 0);
  auto traj_y = xt::view(trajectory, xt::all(), 1);
  auto traj_yaws = xt::view(trajectory, xt::all(), 2);

  xt::noalias(traj_yaws) =
    utils::normalize_angles(xt::cumsum(wz * settings_.model_dt, 0) + initial_yaw);

  auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

  const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

  xt::noalias(xt::view(yaw_cos, 0)) = std::cos(initial_yaw);
  xt::noalias(xt::view(yaw_sin, 0)) = std::sin(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
  xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

  auto && dx = xt::eval(vx * yaw_cos);
  auto && dy = xt::eval(vx * yaw_sin);

  if (isHolonomic()) {
    dx = dx - vy * yaw_sin;
    dy = dy + vy * yaw_cos;
  }

  xt::noalias(traj_x) = state_.pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 0);
  xt::noalias(traj_y) = state_.pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 0);
}

void Optimizer::integrateStateVelocities(
  models::Trajectories & trajectories,
  const models::State & state) const
{
  const double initial_yaw = tf2::getYaw(state_.pose.pose.orientation);

  xt::noalias(trajectories.yaws) =
    utils::normalize_angles(xt::cumsum(state.wz * settings_.model_dt, 1) + initial_yaw);

  const auto yaws_cutted = xt::view(trajectories.yaws, xt::all(), xt::range(0, -1));

  auto && yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = std::cos(initial_yaw);
  xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = std::sin(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) = xt::cos(yaws_cutted);
  xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) = xt::sin(yaws_cutted);

  auto && dx = xt::eval(state.vx * yaw_cos);
  auto && dy = xt::eval(state.vx * yaw_sin);

  if (isHolonomic()) {
    dx = dx - state.vy * yaw_sin;
    dy = dy + state.vy * yaw_cos;
  }

  xt::noalias(trajectories.x) = state_.pose.pose.position.x +
    xt::cumsum(dx * settings_.model_dt, 1);
  xt::noalias(trajectories.y) = state_.pose.pose.position.y +
    xt::cumsum(dy * settings_.model_dt, 1);
}

xt::xtensor<float, 2> Optimizer::getOptimizedTrajectory()
{
  auto && state =
    xt::xtensor<float, 2>::from_shape({settings_.time_steps, isHolonomic() ? 3u : 2u});
  auto && trajectories = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3});

  xt::noalias(xt::view(state, xt::all(), 0)) = action_sequence_.vx;
  xt::noalias(xt::view(state, xt::all(), 1)) = action_sequence_.wz;

  if (isHolonomic()) {
    xt::noalias(xt::view(state, xt::all(), 2)) = action_sequence_.vy;
  }

  integrateStateVelocities(trajectories, state);
  return std::move(trajectories);
}

void Optimizer::updateControlSequence()
{
  // TODO costs_ on action sequence
  // maybe should be a critic function, maybe power of 1 like critics or 2 like paper?
  // sum ((a_t - a_{t-1}) * w * (a_t - a_{t-1}))

  // TODO maybe why not working: using vx derivative of `c` for trajectories/scoring, not of `a`?

  // TODO trajectories still going nutty - not stopping at goal point or even on path at times, seems overly fast/not distributed as much
  
  // Contact SMMPI guys, incentives in citations to help. Plus 10,000+ devs / braggings rights in ROS. So close and solves a remaining issue
  // Look at figures
  float weight = 0.8;
  const auto avx1 = xt::view(state_.avx,  xt::all(), xt::range(_, -1));
  const auto avx2 = xt::view(state_.avx, xt::all(), xt::range(1, _));
  const auto d_avx = xt::fabs(avx2 - avx1);

  const auto awz1 = xt::view(state_.awz, xt::all(), xt::range(_, -1));
  const auto awz2 = xt::view(state_.awz, xt::all(), xt::range(1, _));
  const auto d_awz = xt::fabs(awz2 - awz1);

  if (isHolonomic()) {
    const auto avy1 = xt::view(state_.avy, xt::all(), xt::range(_, -1));
    const auto avy2 = xt::view(state_.avy, xt::all(), xt::range(1, _));
    const auto d_avy = xt::fabs(avy2 - avy1);
    costs_ += xt::sum(d_avx * d_avx + d_avy * d_avy + d_awz * d_awz, {1}, immediate) * weight;
  } else {
    costs_ += xt::sum(d_avx * d_avx + d_awz * d_awz, {1}, immediate) * weight;
  }

  auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
  auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
  auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
  auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

  xt::noalias(control_sequence_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
  xt::noalias(control_sequence_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
  xt::noalias(control_sequence_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
}

void Optimizer::updateActionSequence()
{
  xt::noalias(action_sequence_.vx) = action_sequence_.vx + (control_sequence_.vx * settings_.model_dt);
  xt::noalias(action_sequence_.vy) = action_sequence_.vy + (control_sequence_.vy * settings_.model_dt);
  xt::noalias(action_sequence_.wz) = action_sequence_.wz + (control_sequence_.wz * settings_.model_dt);

 auto & s = settings_;
  if (isHolonomic()) {
    action_sequence_.vy = xt::clip(action_sequence_.vy, -s.constraints.vy, s.constraints.vy);
  }

  action_sequence_.vx = xt::clip(action_sequence_.vx, s.constraints.vx_min, s.constraints.vx_max);
  action_sequence_.wz = xt::clip(action_sequence_.wz, -s.constraints.wz, s.constraints.wz);

}

geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
  const builtin_interfaces::msg::Time & stamp)
{
  unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

  auto vx = control_sequence_.vx(offset);
  auto wz = control_sequence_.wz(offset);

  if (isHolonomic()) {
    auto vy = control_sequence_.vy(offset);
    return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
  }

  return utils::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());
}

geometry_msgs::msg::TwistStamped Optimizer::getActionFromSequenceAsTwist(
  const builtin_interfaces::msg::Time & stamp)
{
  unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

  auto vx = action_sequence_.vx(offset);
  auto wz = action_sequence_.wz(offset);

  if (isHolonomic()) {
    auto vy = action_sequence_.vy(offset);
    return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
  }

  return utils::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());
}

void Optimizer::setMotionModel(const std::string & model)
{
  if (model == "DiffDrive") {
    motion_model_ = std::make_unique<DiffDriveMotionModel>();
  } else if (model == "Omni") {
    motion_model_ = std::make_unique<OmniMotionModel>();
  } else if (model == "Ackermann") {
    motion_model_ = std::make_unique<AckermannMotionModel>(parameters_handler_);
  } else {
    throw std::runtime_error(
            std::string(
              "Model %s is not valid! Valid options are DiffDrive, Omni, "
              "or Ackermann",
              model.c_str()));
  }
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
{
  auto & s = settings_;
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    s.constraints.vx_max = s.base_constraints.vx_max;
    s.constraints.vx_min = s.base_constraints.vx_min;
    s.constraints.vy = s.base_constraints.vy;
    s.constraints.wz = s.base_constraints.wz;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      double ratio = speed_limit / 100.0;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    } else {
      // Speed limit is expressed in absolute value
      double ratio = speed_limit / s.base_constraints.vx_max;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
  }
}

models::Trajectories & Optimizer::getGeneratedTrajectories()
{
  return generated_trajectories_;
}

}  // namespace mppi
