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

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_core/goal_checker.hpp"

#include "mppic/models/control_sequence.hpp"
#include "mppic/models/critic_function_data.hpp"

namespace mppi::utils
{

template<typename T, typename S>
geometry_msgs::msg::TwistStamped toTwistStamped(
  const T & velocities, models::ControlSequnceIdxes idx,
  const bool & is_holonomic, const S & stamp, const std::string & frame)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame;
  twist.header.stamp = stamp;
  twist.twist.linear.x = velocities(idx.vx());
  twist.twist.angular.z = velocities(idx.wz());

  if (is_holonomic) {
    twist.twist.linear.y = velocities(idx.vy());
  }

  return twist;
}

inline xt::xtensor<double, 2> toTensor(const nav_msgs::msg::Path & path)
{
  size_t path_size = path.poses.size();
  static constexpr size_t last_dim_size = 3;

  xt::xtensor<double, 2> points = xt::empty<double>({path_size, last_dim_size});

  for (size_t i = 0; i < path_size; ++i) {
    points(i, 0) = static_cast<double>(path.poses[i].pose.position.x);
    points(i, 1) = static_cast<double>(path.poses[i].pose.position.y);
    points(i, 2) =
      static_cast<double>(tf2::getYaw(path.poses[i].pose.orientation));
  }

  return points;
}

inline bool withinPositionGoalTolerance(
  nav2_core::GoalChecker * goal_checker,
  const geometry_msgs::msg::PoseStamped & robot_pose_arg,
  const xt::xtensor<double, 2> & path)
{
  if (goal_checker) {
    geometry_msgs::msg::Pose pose_tol;
    geometry_msgs::msg::Twist vel_tol;
    goal_checker->getTolerances(pose_tol, vel_tol);

    const double & goal_tol = pose_tol.position.x;

    xt::xtensor<double, 1> robot_pose = {
      static_cast<double>(robot_pose_arg.pose.position.x),
      static_cast<double>(robot_pose_arg.pose.position.y)};
    auto goal_pose = xt::view(path, -1, xt::range(0, 2));

    double dist_to_goal = xt::norm_l2(robot_pose - goal_pose, {0})();

    if (dist_to_goal < goal_tol) {
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
  auto theta = xt::eval(xt::fmod(angles + M_PI, 2.0 * M_PI));
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

inline size_t findPathFurthestPoint(const models::CriticFunctionData & data) {
  auto path_points = xt::view(data.path, xt::all(), xt::range(0, 2));

  auto last_points_ext =
    xt::view(data.trajectories, xt::all(), -1, xt::newaxis(), xt::range(0, 2));
  auto distances = xt::norm_l2(last_points_ext - path_points, {2});
  size_t max_id_by_trajectories = 0;
  double min_distance_by_path = std::numeric_limits<double>::max();

  for (size_t i = 0; i < distances.shape(0); i++) {
    size_t min_id_by_path = 0;
    for (size_t j = 0; j < distances.shape(1); j++) {
      if (min_distance_by_path < distances(i, j)) {
        min_distance_by_path = distances(i, j);
        min_id_by_path = j;
      }
    }
    max_id_by_trajectories = std::max(max_id_by_trajectories, min_id_by_path);
  }
  return max_id_by_trajectories;
}


inline void setPathFurthestPointIfNotSet(models::CriticFunctionData & data) {
  if (data.furthest_reached_path_point) {
    return;
  }

  data.furthest_reached_path_point = findPathFurthestPoint(data);
}

inline double distanceFromFurthestToGoal(const models::CriticFunctionData & data) {
  if (!data.furthest_reached_path_point) {
    throw std::runtime_error("Furthest point not computed yet");
  }
  auto path_points = xt::view(data.path, xt::all(), xt::range(0, 2));
  auto furthest_reached_path_point = xt::view(path_points, *data.furthest_reached_path_point, xt::all());
  auto goal_point = xt::view(path_points, -1, xt::all());

  return xt::norm_l2(furthest_reached_path_point - goal_point)();
}

}  // namespace mppi::utils
//
#endif  // MPPIC__UTILS_HPP_
