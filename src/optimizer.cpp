// Copyright 2022 FastSense, Samsung Research
#include "mppic/optimizer.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>

#include "nav2_core/exceptions.hpp"
#include "mppic/models/critic_function_data.hpp"

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
  setOffset();

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
  getParam(s.iteration_count, "iteration_count", 2);
  getParam(s.temperature, "temperature", 0.25);
  getParam(s.base_constraints.vx, "vx_max", 0.5);
  getParam(s.base_constraints.vy, "vy_max", 0.5);
  getParam(s.base_constraints.wz, "wz_max", 1.3);
  getParam(s.sampling_std.vx, "vx_std", 0.2);
  getParam(s.sampling_std.vy, "vy_std", 0.2);
  getParam(s.sampling_std.wz, "wz_std", 1.0);
  getParam(motion_model_name, "motion_model", std::string("DiffDrive"));
  getParentParam(controller_frequency_, "controller_frequency", 0.0);

  s.constraints = s.base_constraints;
  setMotionModel(motion_model_name);
  parameters_handler_->addPostCallback([this]() {reset();});
}

void Optimizer::setOffset()
{
  const double controller_period = 1.0 / controller_frequency_;
  constexpr double eps = 1e-6;

  if (controller_period < settings_.model_dt) {
    RCLCPP_WARN(
      logger_,
      "Controller period is less then model dt, consider setting it equal");
    settings_.control_sequence_shift_offset = 0;
  } else if (abs(controller_period - settings_.model_dt) < eps) {
    RCLCPP_INFO(
      logger_,
      "Controller period is equal to model dt. Control seuqence "
      "shifting is ON");
    settings_.control_sequence_shift_offset = 1;
  } else {
    throw std::runtime_error(
            "Controller period more then model dt, set it equal to model dt");
  }
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
{
  auto & s = settings_;
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    s.constraints.vx = s.base_constraints.vx;
    s.constraints.vy = s.base_constraints.vy;
    s.constraints.wz = s.base_constraints.wz;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      double ratio = speed_limit / 100.0;
      s.constraints.vx = s.base_constraints.vx * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    } else {
      // Speed limit is expressed in absolute value
      double ratio = speed_limit / s.base_constraints.vx;
      s.constraints.vx = speed_limit;
      s.constraints.vy = s.base_constraints.vx * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
  }
}

void Optimizer::reset()
{
  state_.reset(settings_.batch_size, settings_.time_steps);
  state_.getTimeIntervals() = settings_.model_dt;
  control_sequence_.reset(settings_.time_steps);

  RCLCPP_INFO(logger_, "Optimizer reset");
}

geometry_msgs::msg::TwistStamped Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  bool stop_flag = false;

  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generated_trajectories_ =
      generateNoisedTrajectories(robot_pose, robot_speed);

    xt::xtensor<double, 1> costs = xt::zeros<double>({settings_.batch_size});

    auto data =
      models::CriticFunctionData{robot_pose, state_, generated_trajectories_,
      utils::toTensor(plan), goal_checker, costs, stop_flag};
    critic_manager_.evalTrajectoriesScores(data);

    updateControlSequence(costs);
  }

  auto control = getControlFromSequenceAsTwist(
    settings_.control_sequence_shift_offset,
    plan.header.stamp);

  shiftControlSequence();

    
  if (stop_flag) {
    reset();
  }
  return control;
}

void Optimizer::shiftControlSequence()
{
  if (settings_.control_sequence_shift_offset == 0) {
    return;
  }

  using namespace xt::placeholders;  // NOLINT
  control_sequence_.data = xt::roll(control_sequence_.data, -1, 0);

  xt::view(control_sequence_.data, -1, xt::all()) = 
    xt::view(control_sequence_.data, -2, xt::all());
}

xt::xtensor<double, 3> Optimizer::generateNoisedTrajectories(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed)
{
  state_.getControls() = generateNoisedControls();
  applyControlConstraints();
  updateStateVelocities(state_, robot_speed);
  return integrateStateVelocities(state_, robot_pose);
}

xt::xtensor<double, 3> Optimizer::generateNoisedControls() const
{
  auto & s = settings_;
  auto vx_noises = xt::random::randn<double>(
    {s.batch_size, s.time_steps, 1U},
    0.0, s.sampling_std.vx);
  auto wz_noises = xt::random::randn<double>(
    {s.batch_size, s.time_steps, 1U},
    0.0, s.sampling_std.wz);

  if (isHolonomic()) {
    auto vy_noises = xt::random::randn<double>(
      {s.batch_size, s.time_steps, 1U}, 0.0, s.sampling_std.vy);
    return control_sequence_.data +
           xt::concatenate(xt::xtuple(vx_noises, vy_noises, wz_noises), 2);
  }

  return control_sequence_.data +
         xt::concatenate(xt::xtuple(vx_noises, wz_noises), 2);
}

bool Optimizer::isHolonomic() const {return motion_model_->isHolonomic();}

void Optimizer::applyControlConstraints()
{
  auto vx = state_.getControlVelocitiesVX();
  auto wz = state_.getControlVelocitiesWZ();
  auto & s = settings_;

  if (isHolonomic()) {
    auto vy = state_.getControlVelocitiesVY();
    vy = xt::clip(vy, -s.constraints.vy, s.constraints.vy);
  }

  vx = xt::clip(vx, -s.constraints.vx, s.constraints.vx);
  wz = xt::clip(wz, -s.constraints.wz, s.constraints.wz);
}

void Optimizer::updateStateVelocities(
  models::State & state, const geometry_msgs::msg::Twist & robot_speed) const
{
  updateInitialStateVelocities(state, robot_speed);
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(
  models::State & state, const geometry_msgs::msg::Twist & robot_speed) const
{
  xt::view(state.getVelocitiesVX(), xt::all(), 0) = robot_speed.linear.x;
  xt::view(state.getVelocitiesWZ(), xt::all(), 0) = robot_speed.angular.z;

  if (isHolonomic()) {
    xt::view(state.getVelocitiesVY(), xt::all(), 0) = robot_speed.linear.y;
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

span2d Optimizer::getOptimizedTrajectory(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed)
{
  models::State state;
  state.idx.setLayout(motion_model_->isHolonomic());
  state.reset(1U, settings_.time_steps);
  state.getControls() = control_sequence_.data;
  state.getTimeIntervals() = settings_.model_dt;

  updateStateVelocities(state, robot_speed);
  auto trajectory = xt::squeeze(integrateStateVelocities(state, robot_pose));
  return span2d{trajectory.data(), settings_.time_steps, 3};
}

xt::xtensor<double, 3> Optimizer::integrateStateVelocities(
  const models::State & state,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  using namespace xt::placeholders;  // NOLINT

  auto w = state.getVelocitiesWZ();
  double initial_yaw = tf2::getYaw(pose.pose.orientation);
  xt::xtensor<double, 2> yaw =
    xt::cumsum(w * settings_.model_dt, 1) + initial_yaw;
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

  auto x = pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 1);
  auto y = pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 1);

  return xt::concatenate(
    xt::xtuple(
      xt::view(x, xt::all(), xt::all(), xt::newaxis()),
      xt::view(y, xt::all(), xt::all(), xt::newaxis()),
      xt::view(yaw, xt::all(), xt::all(), xt::newaxis())),
    2);
}

void Optimizer::updateControlSequence(const xt::xtensor<double, 1> & costs)
{
  using xt::evaluation_strategy::immediate;

  auto && costs_normalized = costs - xt::amin(costs, immediate);
  auto exponents =
    xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
  auto softmaxes = exponents / xt::sum(exponents, immediate);
  auto softmaxes_expanded =
    xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

  control_sequence_.data =
    xt::sum(state_.getControls() * softmaxes_expanded, 0);
}

auto Optimizer::getControlFromSequence(const unsigned int offset)
{
  return xt::view(control_sequence_.data, offset);
}

geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
  const unsigned int offset, const builtin_interfaces::msg::Time & stamp)
{
  return utils::toTwistStamped(
    getControlFromSequence(offset),
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
    motion_model_ = std::make_unique<AckermannMotionModel>();
  } else {
    throw std::runtime_error(
            std::string(
              "Model %s is not valid! Valid options are DiffDrive, Omni, "
              "or Ackermann",
              model.c_str()));
  }

  state_.idx.setLayout(motion_model_->isHolonomic());
  control_sequence_.idx.setLayout(motion_model_->isHolonomic());
}

span3d Optimizer::getGeneratedTrajectories()
{
  return span3d{generated_trajectories_.data(), settings_.batch_size, settings_.time_steps, 3};
}

}  // namespace mppi
