#pragma once

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_core/exceptions.hpp"

#include "mppi/Optimizer.hpp"

#include "xtensor/xmath.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor/xslice.hpp"
#include "xtensor/xio.hpp"

#include "utils/common.hpp"
#include "utils/geometry.hpp"

#include <limits>

namespace mppi::optimization {

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::evalNextControl(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &robot_speed,
    const nav_msgs::msg::Path &plan)
    -> geometry_msgs::msg::TwistStamped {

  static Tensor costs;
  for (int i = 0; i < iteration_count_; ++i) {
    generated_trajectories_ = generateNoisedTrajectories(robot_pose, robot_speed);
    costs = evalBatchesCosts(generated_trajectories_, plan);
    updateControlSequence(costs);
  }

  return getControlFromSequence(plan.header.stamp, costmap_ros_->getBaseFrameID());
}

template <typename T, typename Tensor, typename Model>
void Optimizer<T, Tensor, Model>::on_configure() {
  costmap_ = costmap_ros_->getCostmap();
  inscribed_radius_ = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
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
  time_steps_ = getParam("time_steps", 20);
  batch_size_ = getParam("batch_size", 300);
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

  obstacle_cost_power_ = getParam("obstacle_cost_power", 2);
  obstacle_cost_weight_ = getParam("obstacle_cost_weight", 10);

  inflation_cost_scaling_factor_ = getParam("inflation_cost_scaling_factor", 3);
  inflation_radius_ = getParam("inflation_radius_", 0.75);

}

template <typename T, typename Tensor, typename Model>
void Optimizer<T, Tensor, Model>::resetBatches() {
  batches_ = xt::zeros<T>({batch_size_, time_steps_, last_dim_size});
  control_sequence_ = xt::zeros<T>({time_steps_, control_dim_size_});
  xt::view(batches_, xt::all(), xt::all(), 4) = model_dt_;
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::generateNoisedTrajectories(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &twist) -> Tensor {

  getBatchesControls() = generateNoisedControlBatches();
  applyControlConstraints();
  setBatchesVelocities(twist);
  return integrateBatchesVelocities(pose);
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
auto Optimizer<T, Tensor, Model>::integrateBatchesVelocities(
    const geometry_msgs::msg::PoseStamped &pose) const -> Tensor {
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
      xt::xtuple(xt::view(x, xt::all(), xt::all(), xt::newaxis()),
                 xt::view(y, xt::all(), xt::all(), xt::newaxis()),
                 xt::view(yaw, xt::all(), xt::all(), xt::newaxis())),
      2);
}

template <typename T, typename Tensor, typename Model>
auto Optimizer<T, Tensor, Model>::evalBatchesCosts(
      const Tensor &batches_of_trajectories, 
      const nav_msgs::msg::Path &path) const
    -> Tensor {


  if (path.poses.empty())
    return xt::zeros<T>({batch_size_});

  using namespace xt::placeholders;

  auto path_points = geometry::toTensor<T>(path);
  auto batch_of_trajectories_points =
      xt::view(batches_of_trajectories, xt::all(), xt::all(), xt::range(0, 2));

  auto &&dists = geometry::distPointsToLineSegments2D(path_points, batch_of_trajectories_points);
  auto &&reference_cost = evalReferenceCost(dists);
  auto &&goal_cost = evalGoalCost(path_points, batch_of_trajectories_points);
  auto &&obstacle_cost = evalObstacleCost(batches_of_trajectories);

  return reference_cost + goal_cost  + obstacle_cost;
}


template <typename T, typename Tensor, typename Model>
template <typename L, typename P>
auto Optimizer<T, Tensor, Model>::evalGoalCost(
    const P &path_points, 
    const L &batches_of_trajectories_points) const {

  auto goal_points = xt::view(path_points, -1, xt::all());
  auto last_timestep_points = xt::view(batches_of_trajectories_points, xt::all(), -1, xt::all());
  auto dim = last_timestep_points.dimension() - 1;

  auto batches_goal_dists = xt::norm_sq(std::move(last_timestep_points) - std::move(goal_points),
                                          {dim});

  return xt::pow(std::move(batches_goal_dists) * goal_cost_weight_, goal_cost_power_);
}

template <typename T, typename Tensor, typename Model>
template <typename D>
auto Optimizer<T, Tensor, Model>::evalReferenceCost(const D &dists) const {
  using xt::evaluation_strategy::immediate;
  auto &&cost = xt::mean(xt::amin(dists, 1, immediate), 1, immediate);

  return xt::pow(std::move(cost) * reference_cost_weight_, reference_cost_power_);
}


template <typename T, typename Tensor, typename Model>
template <typename L>
auto Optimizer<T, Tensor, Model>::evalObstacleCost(const L &batches_of_trajectories_points) const {
  constexpr T collision_cost_value = std::numeric_limits<T>::max();

  Tensor costs = xt::zeros<T>({batch_size_});

  auto minDistToObstacle = [this] (const auto cost) {
    return  (-1.0 / inflation_cost_scaling_factor_) *
      std::log(cost / (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius_;
  };

  for (size_t i = 0; i < static_cast<size_t>(batch_size_); ++i) {
    double min_dist = std::numeric_limits<T>::max();
    bool is_closest_point_inflated = false;
    size_t j = 0;
    for (; j < static_cast<size_t>(time_steps_); ++j) {
      double cost = costAtPose(batches_of_trajectories_points(i, j, 0), batches_of_trajectories_points(i, j, 1));
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
      costs[i] = pow((1.01 * inflation_radius_ - min_dist) * obstacle_cost_weight_, obstacle_cost_power_);
    }

  }

  return costs;
}

template <typename T, typename Tensor, typename Model>
void Optimizer<T, Tensor, Model>::updateControlSequence(const Tensor &costs) {
  auto &&costs_normalized =
      costs - xt::amin(costs, xt::evaluation_strategy::immediate);

  auto exponents = xt::eval(xt::exp(-1 / temperature_ * costs_normalized));

  auto softmaxes =
      exponents / xt::sum(exponents, xt::evaluation_strategy::immediate);

  auto softmaxes_expanded =
      xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

  control_sequence_ = xt::sum(getBatchesControls() * softmaxes_expanded, 0);
}


template <typename T, typename Tensor, typename Model>
bool Optimizer<T, Tensor, Model>::inCollision(unsigned char cost) const {
  if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost != nav2_costmap_2d::NO_INFORMATION;
  } else {
    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }
}

template <typename T, typename Tensor, typename Model>
double Optimizer<T, Tensor, Model>::costAtPose(const double & x, const double & y) const {

  unsigned int mx, my;
  if (not costmap_->worldToMap(x, y, mx, my)) {
    return static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  return static_cast<double>(costmap_->getCost(mx, my));
}

template <typename T, typename Tensor, typename Model>
template <typename S>
auto Optimizer<T, Tensor, Model>::getControlFromSequence(const S &stamp, const std::string &frame)
    -> geometry_msgs::msg::TwistStamped {

  return geometry::toTwistStamped(xt::view(control_sequence_, 0), stamp, frame);
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
