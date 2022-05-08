// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>

#include "nav2_core/exceptions.hpp"

#include "mppic/optimizers/xtensor/optimizer.hpp"
#include "mppic/optimizers/xtensor/models/critic_function_data.hpp"

namespace mppi::xtensor
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
  getParam(s.iteration_count, "iteration_count", 2);
  getParam(s.temperature, "temperature", 0.25);
  getParam(s.base_constraints.vx, "vx_max", 0.5);
  getParam(s.base_constraints.vy, "vy_max", 0.5);
  getParam(s.base_constraints.wz, "wz_max", 1.3);
  getParam(s.sampling_std.vx, "vx_std", 0.2);
  getParam(s.sampling_std.vy, "vy_std", 0.2);
  getParam(s.sampling_std.wz, "wz_std", 1.0);
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

  RCLCPP_INFO(logger_, "Optimizer reset");
}

geometry_msgs::msg::TwistStamped Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  state_.pose = robot_pose;
  state_.speed = robot_speed;

  bool stop_flag = false;

  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generated_trajectories_ =
      generateNoisedTrajectories();

    xt::xtensor<double, 1> costs = xt::zeros<double>({settings_.batch_size});

    auto data =
      models::CriticFunctionData{state_, generated_trajectories_,
      utils::toTensor(plan), goal_checker, costs, stop_flag};
    critic_manager_.evalTrajectoriesScores(data);

    updateControlSequence(costs);
  }

  auto control = getControlFromSequenceAsTwist(plan.header.stamp);

  if (settings_.shift_control_sequence) {
    shiftControlSequence();
  }


  if (stop_flag) {
    reset();
  }
  return control;
}

void Optimizer::shiftControlSequence()
{
  using namespace xt::placeholders;  // NOLINT
  control_sequence_.data = xt::roll(control_sequence_.data, -1, 0);

  xt::view(control_sequence_.data, -1, xt::all()) =
    xt::view(control_sequence_.data, -2, xt::all());
}

xt::xtensor<double, 3> Optimizer::generateNoisedTrajectories()
{
  state_.getControls() = generateNoisedControls();
  applyControlConstraints();
  updateStateVelocities(state_);
  return integrateStateVelocities(state_);
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

span2d Optimizer::getOptimizedTrajectory()
{
  models::State state;
  state.idx.setLayout(motion_model_->isHolonomic());
  state.reset(1U, settings_.time_steps);
  state.getControls() = control_sequence_.data;
  state.getTimeIntervals() = settings_.model_dt;

  updateStateVelocities(state);
  auto trajectory = xt::squeeze(integrateStateVelocities(state));
  return span2d{trajectory.data(), settings_.time_steps, 3};
}

xt::xtensor<double, 3> Optimizer::integrateStateVelocities(const models::State & state) const
{
  using namespace xt::placeholders;  // NOLINT

  auto w = state.getVelocitiesWZ();
  double initial_yaw = tf2::getYaw(state_.pose.pose.orientation);
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

  auto x = state_.pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 1);
  auto y = state_.pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 1);

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

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::xtensor::Optimizer,
  mppi::IOptimizerCore)

}  // namespace mppi
