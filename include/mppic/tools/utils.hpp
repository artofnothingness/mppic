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

#include "mppic/models/action_sequence.hpp"
#include "mppic/models/control_sequence.hpp"
#include "mppic/models/path.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "mppic/critic_data.hpp"

namespace mppi::utils
{

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
  const auto goal_idx = path.x.shape(0);
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
      if (min_distance_by_path < dists(i, j)) {
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

}  // namespace mppi::utils
//
#endif  // MPPIC__UTILS_HPP_
