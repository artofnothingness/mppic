#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

#include "xtensor/xarray.hpp"
#include "xtensor/xnorm.hpp"
#include "xtensor/xview.hpp"

#include <algorithm>
#include <chrono>


namespace mppi::geometry {

template<typename T, typename H>
auto toTwistStamped(
  const T &velocities,
  const H &header)
  -> geometry_msgs::msg::TwistStamped
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = header.frame_id;
  twist.header.stamp = header.stamp;
  twist.twist.linear.x = velocities(0);
  twist.twist.angular.z = velocities(1);
  return twist;
}

template<typename T, typename S>
auto toTwistStamped(
  const T &velocities,
  const S &stamp,
  const std::string &frame)
  -> geometry_msgs::msg::TwistStamped
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame;
  twist.header.stamp = stamp;
  twist.twist.linear.x = velocities(0);
  twist.twist.angular.z = velocities(1);
  return twist;
}

template<typename T>
auto toTensor(const nav_msgs::msg::Path &path)
  -> xt::xtensor<T, 2>
{
  size_t path_size = path.poses.size();
  static constexpr size_t last_dim_size = 3;

  xt::xtensor<T, 2> points = xt::empty<T>({ path_size, last_dim_size });

  for (size_t i = 0; i < path_size; ++i) {
    points(i, 0) = path.poses[i].pose.position.x;
    points(i, 1) = path.poses[i].pose.position.y;
    points(i, 2) = tf2::getYaw(path.poses[i].pose.orientation);
  }

  return points;
}

template<typename T>
auto hypot(const T &p1, const T &p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;

  return std::hypot(dx, dy, dz);
}

template<>
inline auto hypot(
  const geometry_msgs::msg::Pose &lhs,
  const geometry_msgs::msg::Pose &rhs)
{
  return hypot(lhs.position, rhs.position);
}

template<>
inline auto hypot(
  const geometry_msgs::msg::PoseStamped &lhs,
  const geometry_msgs::msg::PoseStamped &rhs)
{
  return hypot(lhs.pose, rhs.pose);
}

// http://paulbourke.net/geometry/pointlineplane/
/**
 * @brief Calculate closest points on batches segments to path points
 *
 * @param batch_of_segments_points batches of sequences of points. Sequences considering as lines
 * @param path 2D data structure with last dim size stands for x, y
 * @return points on line segments closest to batches sequences points
 *      4D data structre of shape [ batch_of_segments_points.shape[0], batch_of_segments_points.shape()[1] - 1,
 *                                  path.shape()[0], path.shape()[1] ]
 */
template<typename P, typename L>
auto closestPointsOnLinesSegment2D(
  P &&path,
  L &&batch_of_segments_points)
{
  using namespace xt::placeholders;
  using T = typename std::decay_t<P>::value_type;

  auto closest_points = xt::xtensor<T, 4>::from_shape(
    { batch_of_segments_points.shape()[0],
      batch_of_segments_points.shape()[1] - 1,
      path.shape()[0],
      path.shape()[1] });

  auto start_line_points = xt::view(batch_of_segments_points, xt::all(), xt::range(_, -1));
  auto end_line_points = xt::view(batch_of_segments_points, xt::all(), xt::range(1, _));

  xt::xtensor<T, 3> diff = end_line_points - start_line_points;
  xt::xtensor<T, 2> sq_norm = xt::norm_sq(
    diff,
    { diff.dimension() - 1 },
    xt::evaluation_strategy::immediate);

  static constexpr double eps = 1e-3;
  for (size_t b = 0; b < closest_points.shape()[0]; ++b) {
    for (size_t t = 0; t < closest_points.shape()[1]; ++t) {
      if (abs(sq_norm(b, t)) < eps) {
        xt::view(closest_points, b, t) = xt::view(start_line_points, b, t);
        continue;
      }

      for (size_t p = 0; p < closest_points.shape()[2]; ++p) {
        T u = ((path(p, 0) - start_line_points(b, t, 0)) * diff(b, t, 0) + (path(p, 1) - start_line_points(b, t, 1)) * diff(b, t, 1)) / sq_norm(b, t);

        if (u <= 0) {
          closest_points(b, t, p, 0) = start_line_points(b, t, 0);
          closest_points(b, t, p, 1) = start_line_points(b, t, 1);
        } else if (u >= 1) {
          closest_points(b, t, p, 0) = end_line_points(b, t, 0);
          closest_points(b, t, p, 1) = end_line_points(b, t, 1);
        } else {
          closest_points(b, t, p, 0) = start_line_points(b, t, 0) + u * diff(b, t, 0);
          closest_points(b, t, p, 1) = start_line_points(b, t, 1) + u * diff(b, t, 1);
        }
      }
    }
  }


  return closest_points;
}

/**
 * @brief Calculate nearest distances between path points and batches segments
 *
 * @param batch_of_segments_points batches of line segments. last dim must have at least 2 dim
 * @param path 2D data structure with last dim must have at least 2 dim
 * @return distances from batches sequences points to line segments
 *      3D data structre of shape [ batch_of_segments_points.shape[0], batch_of_segments_points.shape()[1] - 1,
 *                                  path.shape()[0] ]
 */
template<typename P, typename L>
auto distPointsToLineSegments2D(P &&path, L &&batch_of_segments_points)
{
  auto path_points = xt::view(path, xt::all(), xt::range(0, 2));
  auto batch_of_lines =
    xt::view(batch_of_segments_points, xt::all(), xt::all(), xt::range(0, 2));


  auto &&closest_points = closestPointsOnLinesSegment2D(
    path_points,
    batch_of_lines);

  auto &&diff = std::move(path_points) - std::move(closest_points);
  size_t dim = diff.dimension() - 1;
  return xt::norm_l2(std::move(diff), { dim }, xt::evaluation_strategy::immediate);
}

}// namespace mppi::geometry
