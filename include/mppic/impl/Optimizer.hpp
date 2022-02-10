#pragma once

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "mppic/Optimizer.hpp"
#include "mppic/utils/LineIterator.hpp"
#include "mppic/utils/common.hpp"
#include "mppic/utils/geometry.hpp"

#include "xtensor/xmath.hpp"
#include "xtensor/xrandom.hpp"

#include <limits>

namespace mppi::optimization {

template <typename T, typename Model>
geometry_msgs::msg::TwistStamped Optimizer<T, Model>::evalNextBestControl(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &robot_speed,
    const nav_msgs::msg::Path &plan) {
  for (size_t i = 0; i < iteration_count_; ++i) {
    generated_trajectories_ =
        generateNoisedTrajectories(robot_pose, robot_speed);
    auto costs = evalBatchesCosts(generated_trajectories_, plan, robot_pose);
    updateControlSequence(costs);
  }
  return getControlFromSequence(plan.header.stamp,
                                costmap_ros_->getBaseFrameID());
}

template <typename T, typename Model>
void Optimizer<T, Model>::on_configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
    const std::string &node_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros,
    Model &&model) {
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

template <typename T, typename Model> void Optimizer<T, Model>::getParams() {

  auto getParam = [&](const std::string &param_name, auto default_value) {
    std::string name = node_name_ + '.' + param_name;
    return utils::getParam(name, default_value, parent_);
  };

  model_dt_ = getParam("model_dt", 0.1);
  time_steps_ = static_cast<unsigned int>(getParam("time_steps", 15));
  batch_size_ = static_cast<unsigned int>(getParam("batch_size", 200));
  v_std_ = static_cast<T>(getParam("v_std", 0.1));
  w_std_ = static_cast<T>(getParam("w_std", 0.3));
  v_limit_ = getParam("v_limit", 0.5);
  w_limit_ = getParam("w_limit", 1.3);
  iteration_count_ = static_cast<unsigned int>(getParam("iteration_count", 2));
  temperature_ = getParam("temperature", 0.25);

  reference_cost_power_ =
      static_cast<unsigned int>(getParam("reference_cost_power", 1));

  goal_cost_power_ = static_cast<unsigned int>(getParam("goal_cost_power", 1));
  goal_angle_cost_power_ =
      static_cast<unsigned int>(getParam("goal_angle_cost_power", 1));
  obstacle_cost_power_ =
      static_cast<unsigned int>(getParam("obstacle_cost_power", 2));

  goal_cost_weight_ = static_cast<double>(getParam("goal_cost_weight", 20));

  goal_angle_cost_weight_ =
      static_cast<double>(getParam("goal_angle_cost_weight", 10));

  obstacle_cost_weight_ =
      static_cast<double>(getParam("obstacle_cost_weight", 10));

  reference_cost_weight_ =
      static_cast<double>(getParam("reference_cost_weight", 5));

  inflation_cost_scaling_factor_ =
      getParam("inflation_cost_scaling_factor", 3.0);
  inflation_radius_ = getParam("inflation_radius", 0.75);
  threshold_to_consider_goal_angle_ =
      getParam("threshold_to_consider_goal_angle", 0.30);

  approx_reference_cost_ = getParam("approx_reference_cost", false);
}

template <typename T, typename Model> void Optimizer<T, Model>::reset() {
  batches_ = xt::zeros<T>({batch_size_, time_steps_, batches_last_dim_size_});
  control_sequence_ = xt::zeros<T>({time_steps_, control_dim_size_});
  xt::view(batches_, xt::all(), xt::all(), idxes::dt) = model_dt_;
}

template <typename T, typename Model>
xt::xtensor<T, 3> Optimizer<T, Model>::generateNoisedTrajectories(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &robot_speed) {
  getBatchesControls() = generateNoisedControlBatches();
  applyControlConstraints();
  evalBatchesVelocities(robot_speed, batches_);
  return integrateBatchesVelocities(robot_pose);
}

template <typename T, typename Model>
xt::xtensor<T, 3> Optimizer<T, Model>::generateNoisedControlBatches() const {
  auto v_noises =
      xt::random::randn<T>({batch_size_, time_steps_, 1U}, 0.0, v_std_);
  auto w_noises =
      xt::random::randn<T>({batch_size_, time_steps_, 1U}, 0.0, w_std_);
  return control_sequence_ + xt::concatenate(xt::xtuple(v_noises, w_noises), 2);
}

template <typename T, typename Model>
void Optimizer<T, Model>::applyControlConstraints() {
  auto v = getBatchesControlLinearVelocities();
  auto w = getBatchesControlAngularVelocities();

  v = xt::clip(v, -v_limit_, v_limit_);
  w = xt::clip(w, -w_limit_, w_limit_);
}

template <typename T, typename Model>
void Optimizer<T, Model>::evalBatchesVelocities(
    const geometry_msgs::msg::Twist &robot_speed, auto &batches) const {
  setBatchesInitialVelocities(robot_speed, batches);
  propagateBatchesVelocitiesFromInitials(batches);
}

template <typename T, typename Model>
void Optimizer<T, Model>::setBatchesInitialVelocities(
    const geometry_msgs::msg::Twist &robot_speed, auto &batches) const {
  xt::view(batches, xt::all(), 0, idxes::linear_velocities) =
      robot_speed.linear.x;

  xt::view(batches, xt::all(), 0, idxes::angular_velocities) =
      robot_speed.angular.z;
}

template <typename T, typename Model>
void Optimizer<T, Model>::propagateBatchesVelocitiesFromInitials(
    auto &batches) const {
  using namespace xt::placeholders;

  for (size_t t = 0; t < time_steps_ - 1; t++) {
    auto curr_batch = xt::view(batches, xt::all(), t); // -> batch x 5
    auto next_batch_velocities =
        xt::view(batches, xt::all(), t + 1, xt::range(0, 2)); // batch x 2
    next_batch_velocities = model_(curr_batch);
  }
}

template <typename T, typename Model>
xt::xtensor<T, dims::control_sequence>
Optimizer<T, Model>::evalTrajectoryFromControlSequence(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &robot_speed) const {

  auto batch =
      xt::xtensor<T, 3>::from_shape({1U, time_steps_, batches_last_dim_size_});

  xt::view(batch, 0, xt::all(), xt::range(2, 4)) = control_sequence_;
  xt::view(batch, 0, xt::all(), 4) = model_dt_;

  evalBatchesVelocities(robot_speed, batch);
  auto &&velocities = xt::view(batch, 0, xt::all(), xt::range(0, 2));

  return integrateSequence(velocities, robot_pose);
}

template <typename T, typename Model>
xt::xtensor<T, dims::control_sequence> Optimizer<T, Model>::integrateSequence(
    const auto &sequence, const geometry_msgs::msg::PoseStamped &pose) const {
  using namespace xt::placeholders;

  auto v = xt::view(sequence, xt::all(), idxes::linear_velocities);
  auto w = xt::view(sequence, xt::all(), idxes::angular_velocities);
  auto yaw = xt::cumsum(w * model_dt_, 0);

  auto yaw_offseted = yaw;
  xt::view(yaw_offseted, xt::range(1, _)) =
      xt::eval(xt::view(yaw, xt::range(_, -1)));

  xt::view(yaw_offseted, xt::all()) += tf2::getYaw(pose.pose.orientation);

  auto v_x = v * xt::cos(yaw_offseted);
  auto v_y = v * xt::sin(yaw_offseted);

  auto x = pose.pose.position.x + xt::cumsum(v_x * model_dt_, 0);
  auto y = pose.pose.position.y + xt::cumsum(v_y * model_dt_, 0);

  return xt::concatenate(xt::xtuple(xt::view(x, xt::all(), xt::newaxis()),
                                    xt::view(y, xt::all(), xt::newaxis()),
                                    xt::view(yaw, xt::all(), xt::newaxis())),
                         1);
}

template <typename T, typename Model>
xt::xtensor<T, dims::batches> Optimizer<T, Model>::integrateBatchesVelocities(
    const geometry_msgs::msg::PoseStamped &pose) const {
  using namespace xt::placeholders;

  auto v = getBatchesLinearVelocities();
  auto w = getBatchesAngularVelocities();
  auto yaw = xt::cumsum(w * model_dt_, 1);

  auto yaw_offseted = yaw;
  xt::view(yaw_offseted, xt::all(), xt::range(1, _)) =
      xt::eval(xt::view(yaw, xt::all(), xt::range(_, -1)));
  xt::view(yaw_offseted, xt::all(), 0) = 0;
  xt::view(yaw_offseted, xt::all(), xt::all()) +=
      tf2::getYaw(pose.pose.orientation);

  auto v_x = v * xt::cos(yaw_offseted);
  auto v_y = v * xt::sin(yaw_offseted);

  auto x = pose.pose.position.x + xt::cumsum(v_x * model_dt_, 1);
  auto y = pose.pose.position.y + xt::cumsum(v_y * model_dt_, 1);

  return xt::concatenate(
      xt::xtuple(xt::view(x, xt::all(), xt::all(), xt::newaxis()),
                 xt::view(y, xt::all(), xt::all(), xt::newaxis()),
                 xt::view(yaw, xt::all(), xt::all(), xt::newaxis())),
      2);
}

template <typename T, typename Model>
xt::xtensor<T, 1> Optimizer<T, Model>::evalBatchesCosts(
    const xt::xtensor<T, dims::batches> &batches_of_trajectories,
    const nav_msgs::msg::Path &global_plan,
    const geometry_msgs::msg::PoseStamped &robot_pose) const {
  using namespace xt::placeholders;

  xt::xtensor<T, 1> costs = xt::zeros<T>({batch_size_});

  if (global_plan.poses.empty()) {
    return costs;
  }

  auto &&path_tensor = geometry::toTensor<T>(global_plan);

  approx_reference_cost_
      ? evalApproxReferenceCost(batches_of_trajectories, path_tensor, costs)
      : evalReferenceCost(batches_of_trajectories, path_tensor, costs);

  evalGoalCost(batches_of_trajectories, path_tensor, costs);
  evalGoalAngleCost(batches_of_trajectories, path_tensor, robot_pose, costs);
  evalObstacleCost(batches_of_trajectories, costs);
  return costs;
}

template <typename T, typename Model>
void Optimizer<T, Model>::evalGoalCost(const auto &batches_of_trajectories,
                                       const auto &global_plan,
                                       auto &costs) const {
  const auto goal_points = xt::view(global_plan, -1, xt::range(0, 2));

  auto last_timestep_points =
      xt::view(batches_of_trajectories, xt::all(), -1, xt::range(0, 2));

  auto dim = last_timestep_points.dimension() - 1;

  auto &&batches_last_to_goal_dists =
      xt::norm_l2(std::move(last_timestep_points) - goal_points, {dim});

  costs += xt::pow(std::move(batches_last_to_goal_dists) * goal_cost_weight_,
                   goal_cost_power_);
}

template <typename T, typename Model>
void Optimizer<T, Model>::evalApproxReferenceCost(
    const auto &batches_of_trajectories, const auto &global_plan,
    auto &costs) const {
  auto path_points = xt::view(global_plan, xt::all(), xt::range(0, 2));
  auto batch_of_lines = xt::view(batches_of_trajectories, xt::all(), xt::all(),
                                 xt::newaxis(), xt::range(0, 2));
  auto dists = xt::norm_l2(path_points - batch_of_lines,
                           {batch_of_lines.dimension() - 1});
  auto &&cost = xt::mean(xt::amin(std::move(dists), 1), 1);
  costs +=
      xt::pow(std::move(cost) * reference_cost_weight_, reference_cost_power_);
}

template <typename T, typename Model>
void Optimizer<T, Model>::evalReferenceCost(const auto &batches_of_trajectories,
                                            const auto &global_plan,
                                            auto &costs) const {
  using xt::evaluation_strategy::immediate;

  xt::xtensor<T, 3> path_to_batches_dists =
      geometry::distPointsToLineSegments2D(global_plan,
                                           batches_of_trajectories);

  xt::xtensor<T, 1> cost = xt::mean(
      xt::amin(std::move(path_to_batches_dists), 1, immediate), 1, immediate);

  costs +=
      xt::pow(std::move(cost) * reference_cost_weight_, reference_cost_power_);
}

template <typename T, typename Model>
void Optimizer<T, Model>::evalObstacleCost(
    const auto &batches_of_trajectories_points, auto &costs) const {
  constexpr T collision_cost_value = std::numeric_limits<T>::max() / 2;

  auto minDistToObstacle = [this](const auto cost) {
    return (-1.0 / inflation_cost_scaling_factor_) *
               std::log(static_cast<double>(cost) /
                        (static_cast<double>(
                             nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) -
                         1.0)) +
           inscribed_radius_;
  };

  for (size_t i = 0; i < batch_size_; ++i) {
    double min_dist = std::numeric_limits<double>::max();
    bool inflated = false;
    for (size_t j = 0; j < time_steps_; ++j) {
      std::array<double, 3> pose = {batches_of_trajectories_points(i, j, 0),
                                    batches_of_trajectories_points(i, j, 1),
                                    batches_of_trajectories_points(i, j, 2)};

      auto footprint =
          getOrientedFootprint(pose, costmap_ros_->getRobotFootprint());

      unsigned char cost =
          static_cast<unsigned char>(scoreFootprint(footprint));

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
          pow((1.01 * inflation_radius_ - min_dist) * obstacle_cost_weight_,
              obstacle_cost_power_));
  }
} // namespace mppi::optimization

template <typename T, typename Model>
std::vector<geometry_msgs::msg::Point>
Optimizer<T, Model>::getOrientedFootprint(
    const std::array<double, 3> &robot_pose,
    const std::vector<geometry_msgs::msg::Point> &footprint_spec) const {
  std::vector<geometry_msgs::msg::Point> oriented_footprint;
  oriented_footprint.resize(footprint_spec.size());

  double cost_yaw = cos(robot_pose[2]);
  double sin_yaw = sin(robot_pose[2]);

  for (size_t i = 0; i < footprint_spec.size(); ++i) {
    oriented_footprint[i].x = robot_pose[0] + footprint_spec[i].x * cost_yaw -
                              footprint_spec[i].y * sin_yaw;
    oriented_footprint[i].y = robot_pose[1] + footprint_spec[i].x * sin_yaw +
                              footprint_spec[i].y * cost_yaw;
  }

  return oriented_footprint;
}

template <typename T, typename Model>
void Optimizer<T, Model>::evalGoalAngleCost(
    const auto &batch_of_trajectories, const auto &global_plan,
    const geometry_msgs::msg::PoseStamped &robot_pose, auto &costs) const {
  xt::xtensor<T, 1> tensor_pose = {static_cast<T>(robot_pose.pose.position.x),
                                   static_cast<T>(robot_pose.pose.position.y)};

  auto path_points = xt::view(global_plan, -1, xt::range(0, 2));

  double points_to_goal_dists = xt::norm_l2(tensor_pose - path_points, {0})();

  if (points_to_goal_dists < threshold_to_consider_goal_angle_) {
    auto yaws = xt::view(batch_of_trajectories, xt::all(), xt::all(), 2);
    auto goal_yaw = xt::view(global_plan, -1, 2);

    costs += xt::pow(xt::mean(xt::abs(yaws - goal_yaw), {1}) *
                         goal_angle_cost_weight_,
                     goal_angle_cost_power_);
  }
}

template <typename T, typename Model>
void Optimizer<T, Model>::updateControlSequence(
    const xt::xtensor<T, 1> &costs) {
  using xt::evaluation_strategy::immediate;

  auto &&costs_normalized = costs - xt::amin(costs, immediate);

  auto exponents = xt::eval(xt::exp(-1 / temperature_ * costs_normalized));

  auto softmaxes = exponents / xt::sum(exponents, immediate);

  auto softmaxes_expanded =
      xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

  control_sequence_ = xt::sum(getBatchesControls() * softmaxes_expanded, 0);
}

template <typename T, typename Model>
bool Optimizer<T, Model>::inCollision(unsigned char cost) const {
  if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
           cost != nav2_costmap_2d::NO_INFORMATION;
  } else {
    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }
}

template <typename T, typename Model>
unsigned char Optimizer<T, Model>::costAtPose(const double x,
                                              const double y) const {
  unsigned int mx = 0;
  unsigned int my = 0;
  if (not costmap_->worldToMap(x, y, mx, my)) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  return costmap_->getCost(mx, my);
}

template <typename T, typename Model>
double Optimizer<T, Model>::lineCost(int x0, int x1, int y0, int y1) const {
  double line_cost = 0.0;
  double point_cost = -1.0;

  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {

    point_cost = static_cast<double>(
        costmap_->getCost(static_cast<unsigned int>(line.getX()),
                          static_cast<unsigned int>(line.getY())));

    if (line_cost < point_cost) {
      line_cost = point_cost;
    }
  }

  return line_cost;
}

template <typename T, typename Model>
double Optimizer<T, Model>::scoreFootprint(
    const std::vector<geometry_msgs::msg::Point> &footprint) const {
  unsigned int x0, x1, y0, y1;
  double line_cost = 0.0;
  double footprint_cost = 0.0;

  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {

    if (!costmap_->worldToMap(footprint[i].x, footprint[i].y, x0, y0))
      throw std::runtime_error("Footprint Goes Off Grid.");

    if (!costmap_->worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1))
      throw std::runtime_error("Footprint Goes Off Grid.");

    line_cost = lineCost(static_cast<int>(x0), static_cast<int>(x1),
                         static_cast<int>(y0), static_cast<int>(y1));

    footprint_cost = std::max(line_cost, footprint_cost);
  }

  if (!costmap_->worldToMap(footprint.back().x, footprint.back().y, x0, y0))
    throw std::runtime_error("Footprint Goes Off Grid.");

  if (!costmap_->worldToMap(footprint.front().x, footprint.front().y, x1, y1))
    throw std::runtime_error("Footprint Goes Off Grid.");

  line_cost = lineCost(static_cast<int>(x0), static_cast<int>(x1),
                       static_cast<int>(y0), static_cast<int>(y1));

  footprint_cost = std::max(line_cost, footprint_cost);

  return footprint_cost;
}

template <typename T, typename Model>
geometry_msgs::msg::TwistStamped
Optimizer<T, Model>::getControlFromSequence(const auto &stamp,
                                            const std::string &frame) {
  return geometry::toTwistStamped(xt::view(control_sequence_, 0), stamp, frame);
}

template <typename T, typename Model>
auto Optimizer<T, Model>::getBatchesLinearVelocities() const {
  return xt::view(batches_, xt::all(), xt::all(), idxes::linear_velocities);
}

template <typename T, typename Model>
auto Optimizer<T, Model>::getBatchesAngularVelocities() const {
  return xt::view(batches_, xt::all(), xt::all(), idxes::angular_velocities);
}

template <typename T, typename Model>
auto Optimizer<T, Model>::getBatchesControlLinearVelocities() const {
  return xt::view(batches_, xt::all(), xt::all(),
                  idxes::control_linear_velocities);
}

template <typename T, typename Model>
auto Optimizer<T, Model>::getBatchesControlAngularVelocities() const {
  return xt::view(batches_, xt::all(), xt::all(),
                  idxes::control_angular_velocities);
}

template <typename T, typename Model>
auto Optimizer<T, Model>::getBatchesControls() const {
  return xt::view(batches_, xt::all(), xt::all(),
                  xt::range(std::get<0>(idxes::control_range),
                            std::get<1>(idxes::control_range)));
}

template <typename T, typename Model>
auto Optimizer<T, Model>::getBatchesLinearVelocities() {
  return xt::view(batches_, xt::all(), xt::all(), idxes::linear_velocities);
}

template <typename T, typename Model>
auto Optimizer<T, Model>::getBatchesAngularVelocities() {
  return xt::view(batches_, xt::all(), xt::all(), idxes::angular_velocities);
}

template <typename T, typename Model>
auto Optimizer<T, Model>::getBatchesControlLinearVelocities() {
  return xt::view(batches_, xt::all(), xt::all(),
                  idxes::control_linear_velocities);
}

template <typename T, typename Model>
auto Optimizer<T, Model>::getBatchesControlAngularVelocities() {
  return xt::view(batches_, xt::all(), xt::all(),
                  idxes::control_angular_velocities);
}

template <typename T, typename Model>
auto Optimizer<T, Model>::getBatchesControls() {
  return xt::view(batches_, xt::all(), xt::all(),
                  xt::range(std::get<0>(idxes::control_range),
                            std::get<1>(idxes::control_range)));
}

} // namespace mppi::optimization
