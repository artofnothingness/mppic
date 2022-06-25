// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/optimizer.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace mppi
{

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

  reset();
}

void Optimizer::getParams()
{
  std::string motion_model_name;

  auto & s = settings_;
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter("");
  getParam(s.model_dt, "model_dt", 0.1);
  getParam(s.time_steps, "time_steps", 15);
  getParam(s.batch_size, "batch_size", 400);
  getParam(s.iteration_count, "iteration_count", 1);
  getParam(s.temperature, "temperature", 0.25);
  getParam(s.base_constraints.vx_max, "vx_max", 0.5);
  getParam(s.base_constraints.vx_min, "vx_min", -0.2);
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
  state_.getTimeIntervals() = settings_.model_dt;
  control_sequence_.reset(settings_.time_steps);
  costs_ = xt::zeros<double>({settings_.batch_size});
  generated_trajectories_ = xt::zeros<double>({settings_.batch_size, settings_.time_steps, 3u});
  noises_ =
    xt::zeros<double>({settings_.batch_size, settings_.time_steps, isHolonomic() ? 3u : 2u});

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

  auto control = getControlFromSequenceAsTwist(plan.header.stamp);

  if (settings_.shift_control_sequence) {
    shiftControlSequence();
  }

  return control;
}

void Optimizer::optimize()
{
  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generateNoisedTrajectories();
    critic_manager_.score(critics_data_);
    updateControlSequence();
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

  if (counter > settings_.retry_attempt_limit) {
    counter = 0;
    throw std::runtime_error("Optimizer fail to compute path");
  }

  counter++;
  return true;
}

void Optimizer::prepare(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  state_.pose = robot_pose;
  state_.speed = robot_speed;
  plan_ = utils::toTensor(plan);
  costs_.fill(0);

  critics_data_.fail_flag = false;
  critics_data_.goal_checker = goal_checker;
}

void Optimizer::shiftControlSequence()
{
  using namespace xt::placeholders;  // NOLINT
  control_sequence_.data = xt::roll(control_sequence_.data, -1, 0);

  xt::view(control_sequence_.data, -1, xt::all()) =
    xt::view(control_sequence_.data, -2, xt::all());
}

void Optimizer::generateNoisedTrajectories()
{
  generateNoisedControls();
  applyControlConstraints();
  updateStateVelocities(state_);
  integrateStateVelocities(generated_trajectories_, state_);
}

void Optimizer::generateNoisedControls()
{
  auto & s = settings_;
  auto vx = xt::view(noises_, xt::all(), xt::all(), 0);
  auto wz = xt::view(noises_, xt::all(), xt::all(), isHolonomic() ? 2 : 1);

  vx = xt::random::randn<double>({s.batch_size, s.time_steps}, 0.0, s.sampling_std.vx);
  wz = xt::random::randn<double>({s.batch_size, s.time_steps}, 0.0, s.sampling_std.wz);
  if (isHolonomic()) {
    auto vy = xt::view(noises_, xt::all(), xt::all(), 1);
    vy = xt::random::randn<double>({s.batch_size, s.time_steps}, 0.0, s.sampling_std.vy);
  }

  state_.getControls() = control_sequence_.data + noises_;
}

bool Optimizer::isHolonomic() const {return motion_model_->isHolonomic();}

void Optimizer::applyControlConstraints()
{
  auto & s = settings_;
  auto vx = state_.getControlVelocitiesVX();
  auto wz = state_.getControlVelocitiesWZ();

  if (isHolonomic()) {
    auto vy = state_.getControlVelocitiesVY();
    vy = xt::clip(vy, -s.constraints.vy, s.constraints.vy);
  }

  motion_model_->applyConstraints(state_);

  vx = xt::clip(vx, s.constraints.vx_min, s.constraints.vx_max);
  wz = xt::clip(wz, -s.constraints.wz, s.constraints.wz);
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
  xt::view(state.getVelocitiesVX(), xt::all(), 0) = state_.speed.linear.x;
  xt::view(state.getVelocitiesWZ(), xt::all(), 0) = state_.speed.angular.z;

  if (isHolonomic()) {
    xt::view(state.getVelocitiesVY(), xt::all(), 0) = state_.speed.linear.y;
  }
}

void Optimizer::propagateStateVelocitiesFromInitials(
  models::State & state) const
{
  using namespace xt::placeholders;  // NOLINT

  for (size_t i = 0; i < settings_.time_steps - 1; i++) {
    auto curr_state = xt::view(state.data, xt::all(), i, xt::all());
    auto next_velocities =
      xt::view(
      state.data, xt::all(), i + 1,
      xt::range(state.idx.vbegin(), state.idx.vend()));

    next_velocities = motion_model_->predict(curr_state, state.idx);
  }
}
void Optimizer::integrateStateVelocities(
  xt::xtensor<double, 3> & trajectories,
  const models::State & state) const
{
  using namespace xt::placeholders;  // NOLINT

  auto x = xt::view(trajectories, xt::all(), xt::all(), 0);
  auto y = xt::view(trajectories, xt::all(), xt::all(), 1);
  auto yaw = xt::view(trajectories, xt::all(), xt::all(), 2);

  auto w = state.getVelocitiesWZ();
  double initial_yaw = tf2::getYaw(state_.pose.pose.orientation);
  yaw = xt::cumsum(w * settings_.model_dt, 1) + initial_yaw;
  xt::xtensor<double, 2> yaw_offseted = yaw;

  xt::view(yaw_offseted, xt::all(), xt::range(1, _)) =
    xt::view(yaw, xt::all(), xt::range(_, -1));
  xt::view(yaw_offseted, xt::all(), 0) = initial_yaw;

  auto yaw_cos = xt::eval(xt::cos(yaw_offseted));
  auto yaw_sin = xt::eval(xt::sin(yaw_offseted));

  auto vx = state.getVelocitiesVX();
  auto dx = xt::eval(vx * yaw_cos);
  auto dy = xt::eval(vx * yaw_sin);

  if (isHolonomic()) {
    auto vy = state.getVelocitiesVY();
    dx = dx - vy * yaw_sin;
    dy = dy + vy * yaw_cos;
  }

  x = state_.pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 1);
  y = state_.pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 1);
}

xt::xtensor<double, 2> Optimizer::getOptimizedTrajectory()
{
  models::State state;
  state.idx.setLayout(isHolonomic());
  state.reset(1U, settings_.time_steps);
  state.getControls() = control_sequence_.data;
  state.getTimeIntervals() = settings_.model_dt;

  updateStateVelocities(state);

  auto trajectories = xt::xtensor<double, 3>::from_shape(generated_trajectories_.shape());

  integrateStateVelocities(trajectories, state);
  return xt::squeeze(trajectories);
}

void Optimizer::updateControlSequence()
{
  using xt::evaluation_strategy::immediate;

  auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
  auto exponents =
    xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
  auto softmaxes = exponents / xt::sum(exponents, immediate);
  auto softmaxes_expanded =
    xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

  control_sequence_.data =
    xt::sum(state_.getControls() * softmaxes_expanded, 0);
}

geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
  const builtin_interfaces::msg::Time & stamp)
{
  unsigned int offset = settings_.shift_control_sequence ? 1 : 0;
  return utils::toTwistStamped(
    xt::view(control_sequence_.data, offset),
    control_sequence_.idx, isHolonomic(), stamp,
    costmap_ros_->getBaseFrameID());
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

  state_.idx.setLayout(isHolonomic());
  control_sequence_.idx.setLayout(isHolonomic());
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

xt::xtensor<double, 3> & Optimizer::getGeneratedTrajectories()
{
  return generated_trajectories_;
}

}  // namespace mppi
