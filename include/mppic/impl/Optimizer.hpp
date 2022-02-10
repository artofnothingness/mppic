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
Optimizer<T>::evalNextBestControl(const geometry_msgs::msg::PoseStamped &robot_pose,
                                  const geometry_msgs::msg::Twist &robot_speed,
                                  const nav_msgs::msg::Path &plan) {
  for (size_t i = 0; i < iteration_count_; ++i) {
    generated_trajectories_ = generateNoisedTrajectories(robot_pose, robot_speed);
    auto costs = evalBatchesCosts(generated_trajectories_, plan, robot_pose);
    updateControlSequence(costs);
  }

  return geometry::toTwistStamped(getControlFromSequence(0), plan.header.stamp,
                                  costmap_ros_->getBaseFrameID());
}

template <typename T>
void
Optimizer<T>::on_configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
                           const std::string &node_name,
                           const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros,
                           model_t &&model) {
  parent_ = parent;
  node_name_ = node_name;
  costmap_ros_ = costmap_ros;
  model_ = model;
  costmap_ = costmap_ros_->getCostmap();
  inscribed_radius_ = costmap_ros_->getLayeredCostmap()->getInscribedRadius();

  getParams();
  reset();
  RCLCPP_INFO(logger_, "Configured");
}

template <typename T>
void
Optimizer<T>::getParams() {
  auto setParam = utils::getParamSetter(parent_.get(), node_name_);

  setParam(model_dt_, "model_dt", 0.1);
  setParam(time_steps_, "time_steps", 15);
  setParam(batch_size_, "batch_size", 200);
  setParam(v_std_, "v_std", 0.1);
  setParam(w_std_, "w_std", 0.3);
  setParam(v_limit_, "v_limit", 0.5);
  setParam(w_limit_, "w_limit", 1.3);
  setParam(iteration_count_, "iteration_count", 2);
  setParam(temperature_, "temperature", 0.25);
  setParam(reference_cost_power_, "reference_cost_power", 1);
  setParam(goal_cost_power_, "goal_cost_power", 1);
  setParam(goal_angle_cost_power_, "goal_angle_cost_power", 1);
  setParam(obstacle_cost_power_, "obstacle_cost_power", 2);
  setParam(goal_cost_weight_, "goal_cost_weight", 20);
  setParam(goal_angle_cost_weight_, "goal_angle_cost_weight", 10);
  setParam(obstacle_cost_weight_, "obstacle_cost_weight", 10);
  setParam(reference_cost_weight_, "reference_cost_weight", 5);
  setParam(inflation_cost_scaling_factor_, "inflation_cost_scaling_factor", 3.0);
  setParam(inflation_radius_, "inflation_radius", 0.75);
  setParam(threshold_to_consider_goal_angle_, "threshold_to_consider_goal_angle", 0.30);
  setParam(approx_reference_cost_, "approx_reference_cost", false);
}

template <typename T>
void
Optimizer<T>::reset() {
  state_.data = xt::zeros<T>({batch_size_, time_steps_, batches_last_dim_size_});
  state_.getTimeIntervals() = model_dt_;

  control_sequence_ = xt::zeros<T>({time_steps_, control_dim_size_});
}

template <typename T>
xt::xtensor<T, 3>
Optimizer<T>::generateNoisedTrajectories(const geometry_msgs::msg::PoseStamped &robot_pose,
                                         const geometry_msgs::msg::Twist &robot_speed) {
  state_.getControls() = generateNoisedControlBatches();
  applyControlConstraints();
  evalBatchesVelocities(state_, robot_speed);
  return integrateBatchesVelocities(state_, robot_pose);
}

template <typename T>
xt::xtensor<T, 3>
Optimizer<T>::generateNoisedControlBatches() const {
  auto v_noises = xt::random::randn<T>({batch_size_, time_steps_, 1U}, 0.0, v_std_);
  auto w_noises = xt::random::randn<T>({batch_size_, time_steps_, 1U}, 0.0, w_std_);
  return control_sequence_ + xt::concatenate(xt::xtuple(v_noises, w_noises), 2);
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
Optimizer<T>::evalBatchesVelocities(auto &state,
                                    const geometry_msgs::msg::Twist &robot_speed) const {
  setBatchesInitialVelocities(state, robot_speed);
  propagateBatchesVelocitiesFromInitials(state);
}

template <typename T>
void
Optimizer<T>::setBatchesInitialVelocities(auto &state,
                                          const geometry_msgs::msg::Twist &robot_speed) const {
  xt::view(state.getLinearVelocities(), xt::all(), 0) = robot_speed.linear.x;
  xt::view(state.getAngularVelocities(), xt::all(), 0) = robot_speed.angular.z;
}

template <typename T>
void
Optimizer<T>::propagateBatchesVelocitiesFromInitials(auto &state) const {
  using namespace xt::placeholders;

  for (size_t t = 0; t < time_steps_ - 1; t++) {
    auto curr_state = xt::view(state.data, xt::all(), t);                // -> batch x 5
    auto next_vels = xt::view(state.getVelocities(), xt::all(), t + 1);  // batch x 2

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
  state.getControls() = control_sequence_;
  state.getTimeIntervals() = model_dt_;

  evalBatchesVelocities(state, robot_speed);
  return xt::squeeze(integrateBatchesVelocities(state, robot_pose));
}

template <typename T>
xt::xtensor<T, 3>
Optimizer<T>::integrateBatchesVelocities(const auto &state,
                                         const geometry_msgs::msg::PoseStamped &pose) const {
  using namespace xt::placeholders;

  auto v = state.getLinearVelocities();
  auto w = state.getAngularVelocities();
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

template <typename T>
xt::xtensor<T, 1>
Optimizer<T>::evalBatchesCosts(const xt::xtensor<T, 3> &batches_of_trajectories,
                               const nav_msgs::msg::Path &global_plan,
                               const geometry_msgs::msg::PoseStamped &robot_pose) const {
  using namespace xt::placeholders;

  xt::xtensor<T, 1> costs = xt::zeros<T>({batch_size_});

  if (global_plan.poses.empty()) {
    return costs;
  }

  auto &&path_tensor = geometry::toTensor<T>(global_plan);

  approx_reference_cost_ ? evalApproxReferenceCost(batches_of_trajectories, path_tensor, costs)
                         : evalReferenceCost(batches_of_trajectories, path_tensor, costs);

  evalGoalCost(batches_of_trajectories, path_tensor, costs);
  evalGoalAngleCost(batches_of_trajectories, path_tensor, robot_pose, costs);
  evalObstacleCost(batches_of_trajectories, costs);
  return costs;
}

template <typename T>
void
Optimizer<T>::evalGoalCost(const auto &batches_of_trajectories, const auto &global_plan,
                           auto &costs) const {
  const auto goal_points = xt::view(global_plan, -1, xt::range(0, 2));
  auto last_timestep_points = xt::view(batches_of_trajectories, xt::all(), -1, xt::range(0, 2));
  auto dim = last_timestep_points.dimension() - 1;
  auto &&batches_last_to_goal_dists =
      xt::norm_l2(std::move(last_timestep_points) - goal_points, {dim});

  costs += xt::pow(std::move(batches_last_to_goal_dists) * goal_cost_weight_, goal_cost_power_);
}

template <typename T>
void
Optimizer<T>::evalApproxReferenceCost(const auto &batches_of_trajectories, const auto &global_plan,
                                      auto &costs) const {
  auto path_points = xt::view(global_plan, xt::all(), xt::range(0, 2));
  auto batch_of_lines =
      xt::view(batches_of_trajectories, xt::all(), xt::all(), xt::newaxis(), xt::range(0, 2));
  auto dists = xt::norm_l2(path_points - batch_of_lines, {batch_of_lines.dimension() - 1});
  auto &&cost = xt::mean(xt::amin(std::move(dists), 1), 1);
  costs += xt::pow(std::move(cost) * reference_cost_weight_, reference_cost_power_);
}

template <typename T>
void
Optimizer<T>::evalReferenceCost(const auto &batches_of_trajectories, const auto &global_plan,
                                auto &costs) const {
  using xt::evaluation_strategy::immediate;

  xt::xtensor<T, 3> path_to_batches_dists =
      geometry::distPointsToLineSegments2D(global_plan, batches_of_trajectories);

  xt::xtensor<T, 1> cost =
      xt::mean(xt::amin(std::move(path_to_batches_dists), 1, immediate), 1, immediate);

  costs += xt::pow(std::move(cost) * reference_cost_weight_, reference_cost_power_);
}

template <typename T>
void
Optimizer<T>::evalObstacleCost(const auto &batches_of_trajectories_points, auto &costs) const {
  constexpr T collision_cost_value = std::numeric_limits<T>::max() / 2;

  auto minDistToObstacle = [this](const auto cost) {
    return (-1.0 / inflation_cost_scaling_factor_) *
               std::log(static_cast<double>(cost) /
                        (static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) - 1.0)) +
           inscribed_radius_;
  };

  for (size_t i = 0; i < batch_size_; ++i) {
    double min_dist = std::numeric_limits<double>::max();
    bool inflated = false;
    for (size_t j = 0; j < time_steps_; ++j) {
      std::array<double, 3> pose = {batches_of_trajectories_points(i, j, 0),
                                    batches_of_trajectories_points(i, j, 1),
                                    batches_of_trajectories_points(i, j, 2)};

      auto footprint = getOrientedFootprint(pose, costmap_ros_->getRobotFootprint());

      unsigned char cost = static_cast<unsigned char>(scoreFootprint(footprint));

      if (inCollision(cost)) {
        costs[i] = collision_cost_value;
        inflated = false;
        break;
      }

      if (cost != nav2_costmap_2d::FREE_SPACE) {
        min_dist = std::min(minDistToObstacle(cost), min_dist);
        inflated = true;
      }
    }

    if (inflated)
      costs[i] += static_cast<T>(
          pow((1.01 * inflation_radius_ - min_dist) * obstacle_cost_weight_, obstacle_cost_power_));
  }
}  // namespace mppi::optimization

template <typename T>
std::vector<geometry_msgs::msg::Point>
Optimizer<T>::getOrientedFootprint(
    const std::array<double, 3> &robot_pose,
    const std::vector<geometry_msgs::msg::Point> &footprint_spec) const {
  std::vector<geometry_msgs::msg::Point> oriented_footprint;
  oriented_footprint.resize(footprint_spec.size());

  double cost_yaw = cos(robot_pose[2]);
  double sin_yaw = sin(robot_pose[2]);

  for (size_t i = 0; i < footprint_spec.size(); ++i) {
    oriented_footprint[i].x =
        robot_pose[0] + footprint_spec[i].x * cost_yaw - footprint_spec[i].y * sin_yaw;
    oriented_footprint[i].y =
        robot_pose[1] + footprint_spec[i].x * sin_yaw + footprint_spec[i].y * cost_yaw;
  }

  return oriented_footprint;
}

template <typename T>
void
Optimizer<T>::evalGoalAngleCost(const auto &batch_of_trajectories, const auto &global_plan,
                                const geometry_msgs::msg::PoseStamped &robot_pose,
                                auto &costs) const {
  xt::xtensor<T, 1> tensor_pose = {static_cast<T>(robot_pose.pose.position.x),
                                   static_cast<T>(robot_pose.pose.position.y)};

  auto path_points = xt::view(global_plan, -1, xt::range(0, 2));

  double points_to_goal_dists = xt::norm_l2(tensor_pose - path_points, {0})();

  if (points_to_goal_dists < threshold_to_consider_goal_angle_) {
    auto yaws = xt::view(batch_of_trajectories, xt::all(), xt::all(), 2);
    auto goal_yaw = xt::view(global_plan, -1, 2);

    costs += xt::pow(xt::mean(xt::abs(yaws - goal_yaw), {1}) * goal_angle_cost_weight_,
                     goal_angle_cost_power_);
  }
}

template <typename T>
void
Optimizer<T>::updateControlSequence(const xt::xtensor<T, 1> &costs) {
  using xt::evaluation_strategy::immediate;

  auto &&costs_normalized = costs - xt::amin(costs, immediate);

  auto exponents = xt::eval(xt::exp(-1 / temperature_ * costs_normalized));

  auto softmaxes = exponents / xt::sum(exponents, immediate);

  auto softmaxes_expanded = xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

  control_sequence_ = xt::sum(state_.getControls() * softmaxes_expanded, 0);
}

template <typename T>
bool
Optimizer<T>::inCollision(unsigned char cost) const {
  if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
           cost != nav2_costmap_2d::NO_INFORMATION;
  } else {
    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }
}

template <typename T>
unsigned char
Optimizer<T>::costAtPose(const double x, const double y) const {
  unsigned int mx = 0;
  unsigned int my = 0;
  if (not costmap_->worldToMap(x, y, mx, my)) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  return costmap_->getCost(mx, my);
}

template <typename T>
double
Optimizer<T>::lineCost(int x0, int x1, int y0, int y1) const {
  double line_cost = 0.0;
  double point_cost = -1.0;

  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = static_cast<double>(costmap_->getCost(static_cast<unsigned int>(line.getX()),
                                                       static_cast<unsigned int>(line.getY())));

    if (line_cost < point_cost) {
      line_cost = point_cost;
    }
  }

  return line_cost;
}

template <typename T>
double
Optimizer<T>::scoreFootprint(const std::vector<geometry_msgs::msg::Point> &footprint) const {
  unsigned int x0, x1, y0, y1;
  double line_cost = 0.0;
  double footprint_cost = 0.0;

  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    if (!costmap_->worldToMap(footprint[i].x, footprint[i].y, x0, y0))
      throw std::runtime_error("Footprint Goes Off Grid.");

    if (!costmap_->worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1))
      throw std::runtime_error("Footprint Goes Off Grid.");

    line_cost = lineCost(static_cast<int>(x0), static_cast<int>(x1), static_cast<int>(y0),
                         static_cast<int>(y1));

    footprint_cost = std::max(line_cost, footprint_cost);
  }

  if (!costmap_->worldToMap(footprint.back().x, footprint.back().y, x0, y0))
    throw std::runtime_error("Footprint Goes Off Grid.");

  if (!costmap_->worldToMap(footprint.front().x, footprint.front().y, x1, y1))
    throw std::runtime_error("Footprint Goes Off Grid.");

  line_cost = lineCost(static_cast<int>(x0), static_cast<int>(x1), static_cast<int>(y0),
                       static_cast<int>(y1));

  footprint_cost = std::max(line_cost, footprint_cost);

  return footprint_cost;
}

template <typename T>
auto
Optimizer<T>::getControlFromSequence(unsigned int offset) {
  return xt::view(control_sequence_, offset);
}

}  // namespace mppi::optimization
