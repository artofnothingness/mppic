#pragma once

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_core/exceptions.hpp"

#include "mppi/Optimizer.hpp"

#include "xtensor/xmath.hpp"
#include "xtensor/xrandom.hpp"

#include "utils/common.hpp"
#include "utils/geometry.hpp"

#include <limits>

namespace mppi::optimization {

template<typename T, typename Model>
auto Optimizer<T, Model>::evalNextBestControl(
  const geometry_msgs::msg::PoseStamped &robot_pose,
  const geometry_msgs::msg::Twist &robot_speed,
  const nav_msgs::msg::Path &plan)
  -> geometry_msgs::msg::TwistStamped
{
  for (int i = 0; i < iteration_count_; ++i) {
    generated_trajectories_ = generateNoisedTrajectories(robot_pose, robot_speed);
    auto costs = evalBatchesCosts(generated_trajectories_, plan, robot_pose);
    updateControlSequence(costs);
  }
  return getControlFromSequence(plan.header.stamp, costmap_ros_->getBaseFrameID());
}

template<typename T, typename Model>
auto Optimizer<T, Model>::on_configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
  const std::string &node_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros,
  Model &&model)
  -> void
{
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

template<typename T, typename Model>
auto Optimizer<T, Model>::getParams()
  -> void
{

  auto getParam = [&](const std::string &param_name, auto default_value) {
    std::string name = node_name_ + '.' + param_name;
    return utils::getParam(name, default_value, parent_);
  };

  model_dt_ = getParam("model_dt", 0.1);
  time_steps_ = getParam("time_steps", 15);
  batch_size_ = getParam("batch_size", 200);
  v_std_ = getParam("v_std", 0.1);
  w_std_ = getParam("w_std", 0.3);
  v_limit_ = getParam("v_limit", 0.5);
  w_limit_ = getParam("w_limit", 1.3);
  iteration_count_ = getParam("iteration_count", 2);
  temperature_ = getParam("temperature", 0.25);

  reference_cost_power_ = getParam("reference_cost_power", 1);
  reference_cost_weight_ = getParam("reference_cost_weight", 5);

  goal_cost_power_ = getParam("goal_cost_power", 1.0);
  goal_cost_weight_ = getParam("goal_cost_weight", 20.0);

  goal_angle_cost_power_ = getParam("goal_angle_cost_power", 1.0);
  goal_angle_cost_weight_ = getParam("goal_angle_cost_weight", 10.0);

  obstacle_cost_power_ = getParam("obstacle_cost_power", 2);
  obstacle_cost_weight_ = getParam("obstacle_cost_weight", 10);

  inflation_cost_scaling_factor_ = getParam("inflation_cost_scaling_factor", 3);
  inflation_radius_ = getParam("inflation_radius", 0.75);
  threshold_to_consider_goal_angle_ = getParam("threshold_to_consider_goal_angle", 0.30);

  approx_reference_cost_ = getParam("approx_reference_cost", false);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::reset()
  -> void
{
  batches_ = xt::zeros<T>({ batch_size_, time_steps_, batches_last_dim_size_ });
  control_sequence_ = xt::zeros<T>({ time_steps_, control_dim_size_ });
  xt::view(batches_, xt::all(), xt::all(), 4) = model_dt_;
}

template<typename T, typename Model>
auto Optimizer<T, Model>::generateNoisedTrajectories(
  const geometry_msgs::msg::PoseStamped &robot_pose,
  const geometry_msgs::msg::Twist &robot_speed)
  -> xt::xtensor<T, 3>
{
  getBatchesControls() = generateNoisedControlBatches();
  applyControlConstraints();
  evalBatchesVelocities(robot_speed, batches_);
  return integrateBatchesVelocities(robot_pose);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::generateNoisedControlBatches() const
  -> xt::xtensor<T, 3>
{
  auto v_noises =
    xt::random::randn<T>({ batch_size_, time_steps_, 1 }, 0.0, v_std_);
  auto w_noises =
    xt::random::randn<T>({ batch_size_, time_steps_, 1 }, 0.0, w_std_);
  return control_sequence_ + xt::concatenate(xt::xtuple(v_noises, w_noises), 2);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::applyControlConstraints()
  -> void
{
  auto v = getBatchesControlLinearVelocities();
  auto w = getBatchesControlAngularVelocities();

  v = xt::clip(v, -v_limit_, v_limit_);
  w = xt::clip(w, -w_limit_, w_limit_);
}

template<typename T, typename Model>
void Optimizer<T, Model>::evalBatchesVelocities(
  const geometry_msgs::msg::Twist &robot_speed,
  auto &batches) const
{
  setBatchesInitialVelocities(robot_speed, batches);
  propagateBatchesVelocitiesFromInitials(batches);
}

template<typename T, typename Model>
void Optimizer<T, Model>::setBatchesInitialVelocities(
  const geometry_msgs::msg::Twist &robot_speed,
  auto &batches) const
{
  xt::view(batches, xt::all(), 0, 0) = robot_speed.linear.x;
  xt::view(batches, xt::all(), 0, 1) = robot_speed.angular.z;
}

template<typename T, typename Model>
void Optimizer<T, Model>::propagateBatchesVelocitiesFromInitials(
  auto &batches) const
{
  using namespace xt::placeholders;

  for (int t = 0; t < time_steps_ - 1; t++) {
    auto curr_batch = xt::view(batches, xt::all(), t);// -> batch x 5
    auto next_batch_velocities =
      xt::view(batches, xt::all(), t + 1, xt::range(0, 2));// batch x 2
    next_batch_velocities = model_(curr_batch);
  }
}


template<typename T, typename Model>
auto Optimizer<T, Model>::evalTrajectoryFromControlSequence(
  const geometry_msgs::msg::PoseStamped &robot_pose,
  const geometry_msgs::msg::Twist &robot_speed) const
  -> xt::xtensor<T, 2>
{
  auto batch = xt::xtensor<T, 3>::from_shape(
    { 1,
      static_cast<size_t>(time_steps_),
      static_cast<size_t>(batches_last_dim_size_) });

  xt::view(batch, 0, xt::all(), xt::range(2, 4)) = control_sequence_;
  xt::view(batch, 0, xt::all(), 4) = model_dt_;

  evalBatchesVelocities(robot_speed, batch);
  auto &&velocities = xt::view(batch, 0, xt::all(), xt::range(0, 2));

  return integrateSequence(velocities, robot_pose);
}


template<typename T, typename Model>
auto Optimizer<T, Model>::integrateSequence(
  const auto &sequence,
  const geometry_msgs::msg::PoseStamped &pose) const
  -> xt::xtensor<T, 2>
{
  using namespace xt::placeholders;

  auto v = xt::view(sequence, xt::all(), 0);
  auto w = xt::view(sequence, xt::all(), 1);

  auto yaw = xt::cumsum(w * model_dt_, 0);

  xt::view(yaw, xt::range(1, _)) = xt::view(yaw, xt::range(_, -1));
  xt::view(yaw, xt::all()) += tf2::getYaw(pose.pose.orientation);

  auto v_x = v * xt::cos(yaw);
  auto v_y = v * xt::sin(yaw);

  auto x = pose.pose.position.x + xt::cumsum(v_x * model_dt_, 0);
  auto y = pose.pose.position.y + xt::cumsum(v_y * model_dt_, 0);

  return xt::concatenate(
    xt::xtuple(
      xt::view(x, xt::all(), xt::newaxis()),
      xt::view(y, xt::all(), xt::newaxis()),
      xt::view(yaw, xt::all(), xt::newaxis())),
    1);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::integrateBatchesVelocities(
  const geometry_msgs::msg::PoseStamped &pose) const
  -> xt::xtensor<T, 3>
{
  using namespace xt::placeholders;

  auto v = getBatchesLinearVelocities();
  auto w = getBatchesAngularVelocities();
  auto yaw = xt::cumsum(w * model_dt_, 1);

  xt::view(yaw, xt::all(), xt::range(1, _)) =
    xt::view(yaw, xt::all(), xt::range(_, -1));
  xt::view(yaw, xt::all(), xt::all()) += tf2::getYaw(pose.pose.orientation);

  auto v_x = v * xt::cos(yaw);
  auto v_y = v * xt::sin(yaw);

  auto x = pose.pose.position.x + xt::cumsum(v_x * model_dt_, 1);
  auto y = pose.pose.position.y + xt::cumsum(v_y * model_dt_, 1);

  return xt::concatenate(
    xt::xtuple(
      xt::view(x, xt::all(), xt::all(), xt::newaxis()),
      xt::view(y, xt::all(), xt::all(), xt::newaxis()),
      xt::view(yaw, xt::all(), xt::all(), xt::newaxis())),
    2);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::evalBatchesCosts(
  const xt::xtensor<T, 3> &batches_of_trajectories,
  const nav_msgs::msg::Path &global_plan,
  const geometry_msgs::msg::PoseStamped &robot_pose) const
  -> xt::xtensor<T, 1>
{
  using namespace xt::placeholders;

  xt::xtensor<T, 1> costs = xt::zeros<T>({ batch_size_ });

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


template<typename T, typename Model>
void Optimizer<T, Model>::evalGoalCost(
  const auto &batches_of_trajectories,
  const auto &global_plan,
  auto &costs) const
{
  const auto goal_points =
    xt::view(global_plan, -1, xt::range(0, 2));

  auto last_timestep_points =
    xt::view(batches_of_trajectories, xt::all(), -1, xt::range(0, 2));

  auto dim = last_timestep_points.dimension() - 1;

  auto &&batches_last_to_goal_dists =
    xt::norm_l2(std::move(last_timestep_points) - goal_points, { dim });

  costs += xt::pow(std::move(batches_last_to_goal_dists) * goal_cost_weight_, goal_cost_power_);
}

template<typename T, typename Model>
void Optimizer<T, Model>::evalApproxReferenceCost(
  const auto &batches_of_trajectories,
  const auto &global_plan,
  auto &costs) const
{
  auto path_points = xt::view(global_plan, xt::all(), xt::range(0, 2));
  auto batch_of_lines =
    xt::view(batches_of_trajectories, xt::all(), xt::all(), xt::newaxis(), xt::range(0, 2));
  auto dists = xt::norm_l2(path_points - batch_of_lines, { batch_of_lines.dimension() - 1 });
  auto &&cost = xt::mean(xt::amin(std::move(dists), 1), 1);
  costs += xt::pow(std::move(cost) * reference_cost_weight_, reference_cost_power_);
}

template<typename T, typename Model>
void Optimizer<T, Model>::evalReferenceCost(
  const auto &batches_of_trajectories,
  const auto &global_plan,
  auto &costs) const
{
  using xt::evaluation_strategy::immediate;

  xt::xtensor<T, 3> path_to_batches_dists =
    geometry::distPointsToLineSegments2D(global_plan, batches_of_trajectories);

  xt::xtensor<T, 1> cost =
    xt::mean(xt::amin(std::move(path_to_batches_dists), 1, immediate), 1, immediate);

  costs += xt::pow(std::move(cost) * reference_cost_weight_, reference_cost_power_);
}


template<typename T, typename Model>
void Optimizer<T, Model>::evalObstacleCost(
  const auto &batches_of_trajectories_points,
  auto &costs) const
{
  constexpr T collision_cost_value = std::numeric_limits<T>::max() / 2;

  auto minDistToObstacle = [this](const auto cost) {
    return (-1.0 / inflation_cost_scaling_factor_) * std::log(cost / (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius_;
  };

  for (size_t i = 0; i < static_cast<size_t>(batch_size_); ++i) {
    double min_dist = std::numeric_limits<T>::max();
    bool is_closest_point_inflated = false;
    size_t j = 0;
    for (; j < static_cast<size_t>(time_steps_); ++j) {
      double cost = costAtPose(
        batches_of_trajectories_points(i, j, 0),
        batches_of_trajectories_points(i, j, 1));

      if (inCollision(cost)) {
        costs[i] = collision_cost_value;
        is_closest_point_inflated = false;
        break;
      } else {
        if (cost != nav2_costmap_2d::FREE_SPACE) {
          double dist = minDistToObstacle(cost);
          if (dist < min_dist) {
            is_closest_point_inflated = true;
            min_dist = dist;
          }
        }
      }
    }

    if (is_closest_point_inflated) {
      costs[i] += pow(
        (1.01 * inflation_radius_ - min_dist) * obstacle_cost_weight_,
        obstacle_cost_power_);
    }
  }
}

template<typename T, typename Model>
void Optimizer<T, Model>::evalGoalAngleCost(
  const auto &batch_of_trajectories,
  const auto &global_plan,
  const geometry_msgs::msg::PoseStamped &robot_pose,
  auto &costs) const
{
  xt::xtensor<T, 1> tensor_pose = { static_cast<T>(robot_pose.pose.position.x),
    static_cast<T>(robot_pose.pose.position.y) };

  auto path_points = xt::view(global_plan, -1, xt::range(0, 2));

  T points_to_goal_dists = xt::norm_l2(tensor_pose - path_points, { 0 })();

  if (points_to_goal_dists < threshold_to_consider_goal_angle_) {
    auto yaws = xt::view(batch_of_trajectories, xt::all(), xt::all(), 2);
    auto goal_yaw = xt::view(global_plan, -1, 2);

    costs += xt::pow(
      xt::mean(xt::abs(yaws - goal_yaw), { 1 }) * goal_angle_cost_weight_,
      goal_angle_cost_power_);
  }
}

template<typename T, typename Model>
auto Optimizer<T, Model>::updateControlSequence(const xt::xtensor<T, 1> &costs)
  -> void
{
  using xt::evaluation_strategy::immediate;

  auto &&costs_normalized =
    costs - xt::amin(costs, immediate);

  auto exponents = xt::eval(xt::exp(-1 / temperature_ * costs_normalized));

  auto softmaxes =
    exponents / xt::sum(exponents, immediate);

  auto softmaxes_expanded =
    xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

  control_sequence_ = xt::sum(getBatchesControls() * softmaxes_expanded, 0);
}


template<typename T, typename Model>
bool Optimizer<T, Model>::inCollision(unsigned char cost) const
{
  if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost != nav2_costmap_2d::NO_INFORMATION;
  } else {
    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }
}

template<typename T, typename Model>
auto Optimizer<T, Model>::costAtPose(const double &x, const double &y) const
  -> double
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (not costmap_->worldToMap(x, y, mx, my)) {
    return static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  return static_cast<double>(costmap_->getCost(mx, my));
}

template<typename T, typename Model>
auto Optimizer<T, Model>::getControlFromSequence(const auto &stamp, const std::string &frame)
  -> geometry_msgs::msg::TwistStamped
{

  return geometry::toTwistStamped(xt::view(control_sequence_, 0), stamp, frame);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::getBatchesLinearVelocities() const
{
  return xt::view(batches_, xt::all(), xt::all(), 0);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::getBatchesAngularVelocities() const
{
  return xt::view(batches_, xt::all(), xt::all(), 1);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::getBatchesControlLinearVelocities() const
{
  return xt::view(batches_, xt::all(), xt::all(), 2);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::getBatchesControlAngularVelocities() const
{
  return xt::view(batches_, xt::all(), xt::all(), 3);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::getBatchesControls() const
{
  return xt::view(batches_, xt::all(), xt::all(), xt::range(2, 4));
}

template<typename T, typename Model>
auto Optimizer<T, Model>::getBatchesLinearVelocities()
{
  return xt::view(batches_, xt::all(), xt::all(), 0);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::getBatchesAngularVelocities()
{
  return xt::view(batches_, xt::all(), xt::all(), 1);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::getBatchesControls()
{
  return xt::view(batches_, xt::all(), xt::all(), xt::range(2, 4));
}

template<typename T, typename Model>
auto Optimizer<T, Model>::getBatchesControlLinearVelocities()
{
  return xt::view(batches_, xt::all(), xt::all(), 2);
}

template<typename T, typename Model>
auto Optimizer<T, Model>::getBatchesControlAngularVelocities()
{
  return xt::view(batches_, xt::all(), xt::all(), 3);
}

}// namespace mppi::optimization
