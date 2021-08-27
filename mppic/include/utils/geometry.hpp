#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

#include "xtensor/xarray.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xnorm.hpp"
#include "xtensor/xstrided_view.hpp"
#include "xtensor/xview.hpp"
#include <xtensor/xio.hpp>

#include "algorithm"
#include <chrono>


namespace mppi::geometry {

template <typename T, typename H>
geometry_msgs::msg::TwistStamped toTwistStamped(const T &velocities,
                                                const H &header) {
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = header.frame_id;
  twist.header.stamp = header.stamp;
  twist.twist.linear.x = velocities(0);
  twist.twist.angular.z = velocities(1);
  return twist;
}

template <typename T, typename S>
geometry_msgs::msg::TwistStamped toTwistStamped(const T &velocities, const S &stamp, 
                                                const std::string &frame) {
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame;
  twist.header.stamp = stamp;
  twist.twist.linear.x = velocities(0);
  twist.twist.angular.z = velocities(1);
  return twist;
}

template <typename T, typename Tensor = xt::xarray<T>>
Tensor toTensor(const nav_msgs::msg::Path &path) {
  size_t size = path.poses.size();
  static constexpr size_t last_dim_size = 3;

  Tensor points = xt::empty<T>({size, last_dim_size});

  for (size_t i = 0; i < size; ++i) {
    points(i, 0) = path.poses[i].pose.position.x;
    points(i, 1) = path.poses[i].pose.position.y;
    points(i, 2) = tf2::getYaw(path.poses[i].pose.orientation);
  }

  return points;
}

template <typename T>
auto hypot(const T &p1, const T &p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;

  return std::hypot(dx, dy, dz);
}

template <>
inline auto hypot(const geometry_msgs::msg::Pose &lhs,
                  const geometry_msgs::msg::Pose &rhs) {
  return hypot(lhs.position, rhs.position);
}

template <>
inline auto hypot(const geometry_msgs::msg::PoseStamped &lhs,
                  const geometry_msgs::msg::PoseStamped &rhs) {
  return hypot(lhs.pose, rhs.pose);
}

// http://paulbourke.net/geometry/pointlineplane/
/**
 * @brief Calculate closest between path points and batch lines segments
 *
 * @param batch_of_lines batches of sequences of points. Sequences considering as lines
 * @param path_points 2D data structure with last dim size stands for x, y
 * @return points on line segments closest to batches sequences points
 *      4D data structre of shape [ points.shape[0], points.shape()[1] - 1,
 *      line_points.shape()[0], line_points.shape()[1] ]
 */
template <typename P, typename L>
auto closestPointsOnLinesSegment2D(const P &path_points,
                                   const L &batch_of_lines) {
  using namespace xt::placeholders;
  using T = typename std::decay_t<P>::value_type;
  using Tensor = xt::xarray<T>;

  auto closest_points = Tensor::from_shape({batch_of_lines.shape()[0],
                                            batch_of_lines.shape()[1] - 1,
                                            path_points.shape()[0],
                                            path_points.shape()[1]});

  auto start_line_points = xt::view(batch_of_lines, xt::all(), xt::range(_, -1));
  auto end_line_points = xt::view(batch_of_lines, xt::all(), xt::range(1, _));

  Tensor diff = end_line_points - start_line_points;
  Tensor sq_norm = xt::norm_sq(
      diff, {diff.dimension() - 1}, xt::evaluation_strategy::immediate);

  static constexpr double eps = 1e-3;
  for (size_t b = 0; b < closest_points.shape()[0]; ++b) {
    for (size_t t = 0; t < closest_points.shape()[1]; ++t) {
      if (abs(sq_norm.unchecked(b, t)) < eps) {
        xt::view(closest_points, b, t) = xt::view(start_line_points, b, t);
        continue;
      }

      for (size_t p = 0; p < closest_points.shape()[2]; ++p) {
        auto curr_closest_pt = xt::view(closest_points, b, t, p);
        auto curr_pt = xt::view(path_points, p);
        auto curr_start_pt = xt::view(start_line_points, b, t);
        auto curr_end_pt = xt::view(end_line_points, b, t);
        auto curr_line_diff = xt::view(diff, b, t);

        T u = ((curr_pt.unchecked(0) - curr_start_pt.unchecked(0)) * curr_line_diff.unchecked(0) +
               (curr_pt.unchecked(1) - curr_start_pt.unchecked(1)) * curr_line_diff.unchecked(1)) /
              sq_norm.unchecked(b, t);

        if (u <= 0)
          curr_closest_pt = curr_start_pt;
        else if (u >= 1)
          curr_closest_pt = curr_end_pt;
        else
          curr_closest_pt = curr_start_pt + u * curr_line_diff;
      }
    }
  }


  return closest_points;
}

/**
 * @brief Calculate distances from batches of point sequences to line segments
 *
 * @param batch_of_trajectories batches of line segments. last dim must have at least 2 dim
 * @param path_tensor 2D data structure with last dim must have at least 2 dim
 * @return distances from batches sequences points to line segments
 *      3D data structre of shape [ batch_of_lines.shape[0], batch_of_lines.shape()[1] - 1,
 *      path_points.shape()[0] ]
 */
template <typename P, typename L>
auto distPointsToLineSegments2D(const P &path_tensor, const L &batches_of_trajectories) {

  auto path_points = xt::view(path_tensor, xt::all(), xt::range(0, 2));
  auto batch_of_lines =
      xt::view(batches_of_trajectories, xt::all(), xt::all(), xt::range(0, 2));


  auto &&closest_points = closestPointsOnLinesSegment2D(path_points, 
                                                        batch_of_lines);

  auto &&diff = path_points - std::move(closest_points);
  size_t dim = diff.dimension() - 1;
  return xt::norm_l2(std::move(diff), {dim});
}

} // namespace mppi::geometry

