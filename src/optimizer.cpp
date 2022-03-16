#include "mppic/optimizer.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>

#include "magic_enum.hpp"
#include "mppic/optimization/motion_model.hpp"
#include "mppic/utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace mppi {

void Optimizer::initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string& name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, model_t model) {
  parent_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  model_ = model;
  costmap_ = costmap_ros_->getCostmap();

  auto node = parent_.lock();
  logger_ = node->get_logger();

  getParams();
  setOffset();

  critic_manager_.on_configure(parent_, name_, costmap_ros_);

  reset();
}

void Optimizer::getParams() {
  auto node = parent_.lock();

  auto getParam = utils::getParamGetter(node, name_);
  auto getParentParam = utils::getParamGetter(node, "");

  getParam(model_dt_, "model_dt", 0.1);
  getParam(time_steps_, "time_steps", 15);
  getParam(batch_size_, "batch_size", 200);
  getParam(iteration_count_, "iteration_count", 2);
  getParam(temperature_, "temperature", 0.25);

  getParam(vx_max_, "vx_max", 0.5);
  getParam(vy_max_, "vy_max", 1.3);
  getParam(wz_max_, "wz_max", 1.3);
  getParam(vx_std_, "vx_std", 0.1);
  getParam(vy_std_, "vy_std", 0.1);
  getParam(wz_std_, "wz_std", 0.3);
  getParam(control_sequence_shift_offset_, "control_sequence_shift_offset", 1);

  getParentParam(controller_frequency_, "controller_frequency", 0.0);

  std::string motion_model_name;
  getParam(motion_model_name, "motion_model", std::string("DiffDrive"));
  auto motion_model = magic_enum::enum_cast<MotionModel>(motion_model_name);

  if (motion_model.has_value()) {
    setMotionModel(motion_model.value());
  } else {
    RCLCPP_WARN(logger_, "Motion model is unknown, use default/previous");
  }
}

void Optimizer::setOffset() {
  const double controller_period = 1.0 / controller_frequency_;
  constexpr double eps = 1e-6;

  if (controller_period < model_dt_) {
    RCLCPP_WARN(
        logger_,
        "Controller period is less then model dt, consider setting it equal");
    control_sequence_shift_offset_ = 0;
  } else if (abs(controller_period - model_dt_) < eps) {
    RCLCPP_INFO(logger_,
                "Controller period is equal to model dt. Control seuqence "
                "shifting is ON");
    control_sequence_shift_offset_ = 1;
  } else {
    throw std::runtime_error("Controller period more then model dt, set it equal to model dt");
  }
}

void Optimizer::reset() {
  state_.reset(batch_size_, time_steps_);
  state_.getTimeIntervals() = model_dt_;
  control_sequence_.reset(time_steps_);
}

geometry_msgs::msg::TwistStamped Optimizer::evalControl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    const nav_msgs::msg::Path& plan,
    nav2_core::GoalChecker* goal_checker) {
  for (size_t i = 0; i < iteration_count_; ++i) {
    generated_trajectories_ =
        generateNoisedTrajectories(robot_pose, robot_speed);
    auto&& costs = critic_manager_.evalTrajectoriesScores(
        generated_trajectories_, plan, robot_pose, goal_checker);
    updateControlSequence(costs);
  }

  auto control = getControlFromSequenceAsTwist(0, plan.header.stamp);

  shiftControlSequence();
  return control;
}

void Optimizer::shiftControlSequence() {
  if (control_sequence_shift_offset_ == 0) {
    return;
  }

  using namespace xt::placeholders;
  xt::view(control_sequence_.data,
           xt::range(_, -control_sequence_shift_offset_), xt::all()) =
      xt::view(control_sequence_.data,
               xt::range(control_sequence_shift_offset_, _), xt::all());
}

xt::xtensor<double, 3> Optimizer::generateNoisedTrajectories(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const geometry_msgs::msg::Twist& robot_speed) {
  state_.getControls() = generateNoisedControls();
  applyControlConstraints();
  updateStateVelocities(state_, robot_speed);
  return integrateStateVelocities(state_, robot_pose);
}

xt::xtensor<double, 3> Optimizer::generateNoisedControls() const {
  auto vx_noises =
      xt::random::randn<double>({batch_size_, time_steps_, 1U}, 0.0, vx_std_);
  auto wz_noises =
      xt::random::randn<double>({batch_size_, time_steps_, 1U}, 0.0, wz_std_);

  if (isHolonomic()) {
    auto vy_noises =
        xt::random::randn<double>({batch_size_, time_steps_, 1U}, 0.0, vy_std_);
    return control_sequence_.data +
           xt::concatenate(xt::xtuple(vx_noises, vy_noises, wz_noises), 2);
  }

  return control_sequence_.data +
         xt::concatenate(xt::xtuple(vx_noises, wz_noises), 2);
}

bool Optimizer::isHolonomic() const {
  return mppi::isHolonomic(getMotionModel());
}

void Optimizer::applyControlConstraints() {
  auto vx = state_.getControlVelocitiesVX();
  auto wz = state_.getControlVelocitiesWZ();

  if (isHolonomic()) {
    auto vy = state_.getControlVelocitiesVY();
    vy = xt::clip(vy, -vy_max_, vy_max_);
  }

  vx = xt::clip(vx, -vx_max_, vx_max_);
  wz = xt::clip(wz, -wz_max_, wz_max_);
}

void Optimizer::updateStateVelocities(
    auto& state, const geometry_msgs::msg::Twist& robot_speed) const {
  updateInitialStateVelocities(state, robot_speed);
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(
    auto& state, const geometry_msgs::msg::Twist& robot_speed) const {
  xt::view(state.getVelocitiesVX(), xt::all(), 0) = robot_speed.linear.x;
  xt::view(state.getVelocitiesWZ(), xt::all(), 0) = robot_speed.angular.z;

  if (isHolonomic()) {
    xt::view(state.getVelocitiesVY(), xt::all(), 0) = robot_speed.linear.y;
  }
}

void Optimizer::propagateStateVelocitiesFromInitials(auto& state) const {
  using namespace xt::placeholders;

  for (size_t i = 0; i < time_steps_ - 1; i++) {
    auto curr_state = xt::view(state.data, xt::all(), i);
    auto next_velocities =
        xt::view(state.data, xt::all(), i + 1,
                 xt::range(state.idx.vbegin(), state.idx.vend()));

    next_velocities = model_(curr_state, state.idx);
  }
}

xt::xtensor<double, 2> Optimizer::evalTrajectoryFromControlSequence(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const geometry_msgs::msg::Twist& robot_speed) const {
  optimization::State state;
  state.idx.setLayout(getMotionModel());
  state.reset(1U, time_steps_);
  state.getControls() = control_sequence_.data;
  state.getTimeIntervals() = model_dt_;

  updateStateVelocities(state, robot_speed);
  return xt::squeeze(integrateStateVelocities(state, robot_pose));
}

xt::xtensor<double, 3> Optimizer::integrateStateVelocities(
    const auto& state, const geometry_msgs::msg::PoseStamped& pose) const {
  using namespace xt::placeholders;

  auto w = state.getVelocitiesWZ();
  double initial_yaw = tf2::getYaw(pose.pose.orientation);
  xt::xtensor<double, 2> yaw = xt::cumsum(w * model_dt_, 1) + initial_yaw;
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

  auto x = pose.pose.position.x + xt::cumsum(dx * model_dt_, 1);
  auto y = pose.pose.position.y + xt::cumsum(dy * model_dt_, 1);

  return xt::concatenate(
      xt::xtuple(xt::view(x, xt::all(), xt::all(), xt::newaxis()),
                 xt::view(y, xt::all(), xt::all(), xt::newaxis()),
                 xt::view(yaw, xt::all(), xt::all(), xt::newaxis())),
      2);
}

void Optimizer::updateControlSequence(const xt::xtensor<double, 1>& costs) {
  using xt::evaluation_strategy::immediate;

  auto&& costs_normalized = costs - xt::amin(costs, immediate);
  auto exponents = xt::eval(xt::exp(-1 / temperature_ * costs_normalized));
  auto softmaxes = exponents / xt::sum(exponents, immediate);
  auto softmaxes_expanded =
      xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

  control_sequence_.data =
      xt::sum(state_.getControls() * softmaxes_expanded, 0);
}

geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
    const unsigned int offset, const auto& stamp) {
  return utils::toTwistStamped(getControlFromSequence(offset),
                               control_sequence_.idx, isHolonomic(), stamp,
                               costmap_ros_->getBaseFrameID());
}

auto Optimizer::getControlFromSequence(const unsigned int offset) {
  return xt::view(control_sequence_.data, offset);
}

MotionModel Optimizer::getMotionModel() const { return motion_model_t_; }

void Optimizer::setMotionModel(const MotionModel motion_model) {
  motion_model_t_ = motion_model;
  state_.idx.setLayout(motion_model);
  control_sequence_.idx.setLayout(motion_model);
}

xt::xtensor<double, 3>& Optimizer::getGeneratedTrajectories() {
  return generated_trajectories_;
}

}  // namespace mppi
