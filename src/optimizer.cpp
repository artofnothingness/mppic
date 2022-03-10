#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "mppic/optimization/motion_model.hpp"
#include "mppic/optimizer.hpp"
#include "mppic/utils.hpp"

namespace mppi::optimization {

void Optimizer::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & node_name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, model_t model)
{
  parent_ = parent;
  node_name_ = node_name;
  costmap_ros_ = costmap_ros;
  model_ = model;
  costmap_ = costmap_ros_->getCostmap();

  getParams();
  configureComponents();
  reset();
}

void Optimizer::getParams()
{
  auto node = parent_.lock();
  auto getParam = utils::getParamGetter(node, node_name_);

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

  std::string name;
  getParam(name, "motion_model", std::string("diff"));

  const auto & nmap = MOTION_MODEL_NAMES_MAP;

  if (auto it = nmap.find(name); it != nmap.end()) {
    setMotionModel(it->second);
  } else {
    RCLCPP_INFO(logger_, "Motion model is unknown, use default/previous");
  }
}

// TODO pluginize
void Optimizer::configureComponents()
{
  std::string component_name = node_name_ + ".CriticScorer";
  critic_scorer_.on_configure(parent_, component_name, costmap_ros_);
}

void Optimizer::reset()
{
  state_.reset(batch_size_, time_steps_);
  state_.getTimeIntervals() = model_dt_;
  control_sequence_.reset(time_steps_);
}

geometry_msgs::msg::TwistStamped Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose, const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan)
{
  for (size_t i = 0; i < iteration_count_; ++i) {
    generated_trajectories_ = generateNoisedTrajectories(robot_pose, robot_speed);
    auto costs = critic_scorer_.evalTrajectoriesScores(generated_trajectories_, plan, robot_pose);
    updateControlSequence(costs);
  }

  return getControlFromSequenceAsTwist(0, plan.header.stamp);
}

xt::xtensor<double, 3> Optimizer::generateNoisedTrajectories(
  const geometry_msgs::msg::PoseStamped & robot_pose, const geometry_msgs::msg::Twist & robot_speed)
{
  state_.getControls() = generateNoisedControls();
  applyControlConstraints();
  updateStateVelocities(state_, robot_speed);
  return integrateStateVelocities(state_, robot_pose);
}

xt::xtensor<double, 3> Optimizer::generateNoisedControls() const
{
  auto vx_noises = xt::random::randn<double>({batch_size_, time_steps_, 1U}, 0.0, vx_std_);
  auto wz_noises = xt::random::randn<double>({batch_size_, time_steps_, 1U}, 0.0, wz_std_);

  if (isHolonomic()) {
    auto vy_noises = xt::random::randn<double>({batch_size_, time_steps_, 1U}, 0.0, vy_std_);
    return control_sequence_.data + xt::concatenate(xt::xtuple(vx_noises, vy_noises, wz_noises), 2);
  }

  return control_sequence_.data + xt::concatenate(xt::xtuple(vx_noises, wz_noises), 2);
}

bool Optimizer::isHolonomic() const
{
  return mppi::optimization::isHolonomic(getMotionModel());
}

void Optimizer::applyControlConstraints()
{
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
  auto & state, const geometry_msgs::msg::Twist & robot_speed) const
{
  updateInitialStateVelocities(state, robot_speed);
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(
  auto & state, const geometry_msgs::msg::Twist & robot_speed) const
{
  xt::view(state.getVelocitiesVX(), xt::all(), 0) = robot_speed.linear.x;
  xt::view(state.getVelocitiesWZ(), xt::all(), 0) = robot_speed.angular.z;

  if (isHolonomic()) {
    xt::view(state.getVelocitiesVY(), xt::all(), 0) = robot_speed.linear.y;
  }
}

void Optimizer::propagateStateVelocitiesFromInitials(auto & state) const
{
  using namespace xt::placeholders;

  for (size_t i = 0; i < time_steps_ - 1; i++) {
    auto curr_state = xt::view(state.data, xt::all(), i);
    auto next_velocities =
      xt::view(state.data, xt::all(), i + 1, xt::range(state.idx.vbegin(), state.idx.vend()));

    next_velocities = model_(curr_state, state.idx);
  }
}

xt::xtensor<double, 2> Optimizer::evalTrajectoryFromControlSequence(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed) const
{
  State state;
  state.idx.setLayout(getMotionModel());
  state.reset(1U, time_steps_);
  state.getControls() = control_sequence_.data;
  state.getTimeIntervals() = model_dt_;

  updateStateVelocities(state, robot_speed);
  return xt::squeeze(integrateStateVelocities(state, robot_pose));
}

xt::xtensor<double, 3> Optimizer::integrateStateVelocities(
  const auto & state, const geometry_msgs::msg::PoseStamped & pose) const
{
  using namespace xt::placeholders;

  auto w = state.getVelocitiesWZ();
  double initial_yaw = tf2::getYaw(pose.pose.orientation);
  xt::xtensor<double, 2> yaw = xt::cumsum(w * model_dt_, 1) + initial_yaw;
  xt::xtensor<double, 2> yaw_offseted = yaw;

  xt::view(yaw_offseted, xt::all(), xt::range(1, _)) = xt::view(yaw, xt::all(), xt::range(_, -1));
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
  auto exponents = xt::eval(xt::exp(-1 / temperature_ * costs_normalized));
  auto softmaxes = exponents / xt::sum(exponents, immediate);
  auto softmaxes_expanded = xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

  control_sequence_.data = xt::sum(state_.getControls() * softmaxes_expanded, 0);
}

geometry_msgs::msg::TwistStamped
Optimizer::getControlFromSequenceAsTwist(unsigned int offset, const auto & stamp)
{
  return utils::toTwistStamped(
    getControlFromSequence(offset), control_sequence_.idx, isHolonomic(), stamp,
    costmap_ros_->getBaseFrameID());
}

auto Optimizer::getControlFromSequence(unsigned int offset)
{
  return xt::view(control_sequence_.data, offset);
}

MotionModel Optimizer::getMotionModel() const
{
  return motion_model_t_;
}

void Optimizer::setMotionModel(MotionModel motion_model)
{
  motion_model_t_ = motion_model;
  state_.idx.setLayout(motion_model);
  control_sequence_.idx.setLayout(motion_model);
}

} // namespace mppi::optimization
