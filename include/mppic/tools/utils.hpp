// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#ifndef MPPIC__UTILS_HPP_
#define MPPIC__UTILS_HPP_

#include <algorithm>
#include <chrono>
#include <string>


#include <xtensor/xarray.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "angles/angles.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_core/goal_checker.hpp"

#include "mppic/models/optimizer_settings.hpp"
#include "mppic/models/control_sequence.hpp"
#include "mppic/models/path.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "mppic/critic_data.hpp"

namespace mppi::utils
{
using xt::evaluation_strategy::immediate;

inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float wz, const builtin_interfaces::msg::Time & stamp, const std::string & frame)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame;
  twist.header.stamp = stamp;
  twist.twist.linear.x = vx;
  twist.twist.angular.z = wz;

  return twist;
}

inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float vy, float wz, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame)
{
  auto twist = toTwistStamped(vx, wz, stamp, frame);
  twist.twist.linear.y = vy;

  return twist;
}

inline models::Path toTensor(const nav_msgs::msg::Path & path)
{
  auto result = models::Path{};
  result.reset(path.poses.size());

  for (size_t i = 0; i < path.poses.size(); ++i) {
    result.x(i) = path.poses[i].pose.position.x;
    result.y(i) = path.poses[i].pose.position.y;
    result.yaws(i) = tf2::getYaw(path.poses[i].pose.orientation);
  }

  return result;
}

inline bool withinPositionGoalTolerance(
  nav2_core::GoalChecker * goal_checker,
  const geometry_msgs::msg::Pose & robot,
  const models::Path & path)
{
  const auto goal_idx = path.x.shape(0) - 1;
  const auto goal_x = path.x(goal_idx);
  const auto goal_y = path.y(goal_idx);

  if (goal_checker) {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    goal_checker->getTolerances(pose_tolerance, velocity_tolerance);

    const auto pose_tolerance_sq = pose_tolerance.position.x * pose_tolerance.position.x;

    auto dx = robot.position.x - goal_x;
    auto dy = robot.position.y - goal_y;

    auto dist_sq = dx * dx + dy * dy;

    if (dist_sq < pose_tolerance_sq) {
      return true;
    }
  }

  return false;
}

inline bool withinPositionGoalTolerance(
  float pose_tolerance,
  const geometry_msgs::msg::Pose & robot,
  const models::Path & path)
{
  const auto goal_idx = path.x.shape(0) - 1;
  const auto goal_x = path.x(goal_idx);
  const auto goal_y = path.y(goal_idx);

  const auto pose_tolerance_sq = pose_tolerance * pose_tolerance;

  auto dx = robot.position.x - goal_x;
  auto dy = robot.position.y - goal_y;

  auto dist_sq = dx * dx + dy * dy;

  if (dist_sq < pose_tolerance_sq) {
    return true;
  }

  return false;
}

/**
  * @brief normalize
  *
  * Normalizes the angle to be -M_PI circle to +M_PI circle
  * It takes and returns radians.
  *
  */
template<typename T>
auto normalize_angles(const T & angles)
{
  auto && theta = xt::eval(xt::fmod(angles + M_PI, 2.0 * M_PI));
  return xt::eval(xt::where(theta <= 0.0, theta + M_PI, theta - M_PI));
}

/**
  * @brief shortest_angular_distance
  *
  * Given 2 angles, this returns the shortest angular
  * difference.  The inputs and ouputs are of course radians.
  *
  * The result
  * would always be -pi <= result <= pi.  Adding the result
  * to "from" will always get you an equivelent angle to "to".
  *
  */
template<typename F, typename T>
auto shortest_angular_distance(
  const F & from,
  const T & to)
{
  return normalize_angles(to - from);
}

/**
 * @brief Evaluate furthest point idx of data.path which is
 * nearset to some trajectory in data.trajectories
 */
inline size_t findPathFurthestReachedPoint(const CriticData & data)
{
  const auto traj_x = xt::view(data.trajectories.x, xt::all(), -1, xt::newaxis());
  const auto traj_y = xt::view(data.trajectories.y, xt::all(), -1, xt::newaxis());

  const auto dx = data.path.x - traj_x;
  const auto dy = data.path.y - traj_y;

  const auto dists = dx * dx + dy * dy;

  size_t max_id_by_trajectories = 0;
  double min_distance_by_path = std::numeric_limits<float>::max();

  for (size_t i = 0; i < dists.shape(0); i++) {
    size_t min_id_by_path = 0;
    for (size_t j = 0; j < dists.shape(1); j++) {
      if (min_distance_by_path > dists(i, j)) {
        min_distance_by_path = dists(i, j);
        min_id_by_path = j;
      }
    }
    max_id_by_trajectories = std::max(max_id_by_trajectories, min_id_by_path);
  }
  return max_id_by_trajectories;
}


/**
 * @brief evaluate path furthest point if it is not set
 */
inline void setPathFurthestPointIfNotSet(CriticData & data)
{
  if (!data.furthest_reached_path_point) {
    data.furthest_reached_path_point = findPathFurthestReachedPoint(data);
  }
}

/**
 * @brief evaluate angle from pose (have angle) to point (no angle)
 */
inline double posePointAngle(const geometry_msgs::msg::Pose & pose, double point_x, double point_y)
{
  double pose_x = pose.position.x;
  double pose_y = pose.position.y;
  double pose_yaw = tf2::getYaw(pose.orientation);

  double yaw = atan2(point_y - pose_y, point_x - pose_x);
  return abs(angles::shortest_angular_distance(yaw, pose_yaw));
}

/**
 * @brief Evaluate ratio of data.path which reached by all trajectories in data.trajectories
 */
inline float getPathRatioReached(const CriticData & data)
{
  if (!data.furthest_reached_path_point) {
    throw std::runtime_error("Furthest point not computed yet");
  }

  auto path_points_count = static_cast<float>(data.path.x.shape(0));
  auto furthest_reached_path_point = static_cast<float>(*data.furthest_reached_path_point);
  return furthest_reached_path_point / path_points_count;
}


inline void savitskyGolayFilter(
  models::ControlSequence & control_sequence,
  std::array<mppi::models::Control, 2> & control_history,
  const models::OptimizerSettings & settings)
{
  // Savitzky-Golay Quadratic, 5-point Coefficients
  xt::xarray<float> filter = {-3.0, 12.0, 17.0, 12.0, -3.0};
  filter /= 35.0;

  const unsigned int num_sequences = control_sequence.vx.shape(0);

  // Too short to smooth meaningfully
  if (num_sequences < 10) {
    return;
  }

  auto applyFilter = [&](const xt::xarray<float> & data) -> float {
      return xt::sum(data * filter, {0}, immediate)();
    };

  auto applyFilterOverAxis =
    [&](xt::xtensor<float, 1> & sequence, const float hist_0, const float hist_1) -> void
    {
      unsigned int idx = 0;
      sequence(idx) = applyFilter({
        hist_0,
        hist_1,
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2)});

      idx++;
      sequence(idx) = applyFilter({
        hist_1,
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2)});

      for (idx = 2; idx != num_sequences - 3; idx++) {
        sequence(idx) = applyFilter({
          sequence(idx - 2),
          sequence(idx - 1),
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 2)});
      }

      idx++;
      sequence(idx) = applyFilter({
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 1)});

      idx++;
      sequence(idx) = applyFilter({
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx),
        sequence(idx)});
    };

  // Filter trajectories
  applyFilterOverAxis(control_sequence.vx, control_history[0].vx, control_history[1].vx);
  applyFilterOverAxis(control_sequence.vy, control_history[0].vy, control_history[1].vy);
  applyFilterOverAxis(control_sequence.wz, control_history[0].wz, control_history[1].wz);

  // Update control history
  unsigned int offset = settings.shift_control_sequence ? 1 : 0;
  control_history[0] = control_history[1];
  control_history[1] = {
    control_sequence.vx(offset),
    control_sequence.vy(offset),
    control_sequence.wz(offset)};
}

}  // namespace mppi::utils

#endif  // MPPIC__UTILS_HPP_
