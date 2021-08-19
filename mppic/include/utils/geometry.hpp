#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "algorithm"
#include "xtensor/xarray.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xnorm.hpp"
#include "xtensor/xstrided_view.hpp"

namespace mppi::geometry {

template <typename T, typename H>
geometry_msgs::msg::TwistStamped toTwistStamped(T &&velocities,
                                                const H &header) {
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = header.frame_id;
  twist.header.stamp = header.stamp;
  twist.twist.linear.x = velocities(0);
  twist.twist.angular.z = velocities(1);
  return twist;
}

template <typename T, typename Tensor = xt::xarray<T>>
Tensor toTensor(const nav_msgs::msg::Path &path) {
  size_t size = path.poses.size();
  size_t last_dim_size = 2;

  Tensor points = xt::empty<T>({size, last_dim_size});

  for (size_t i = 0; i < size; ++i) {
    points(i, 0) = path.poses[i].pose.position.x;
    points(i, 1) = path.poses[i].pose.position.y;
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

/**
 * @brief Calculate point on line closest to given point
 *
 * @tparam T point value type
 * @tparam Tensor tensor type consisting points
 * @param point given point: Tensor of shape [P..., 2]
 * @param line_start start line point: Tensor of shape [L..., 2]
 * @param line_end end line point: Tensor of shape [L..., 2]
 * @return points on lines xt::array of shape [L... P..., 2]
 */
template <typename PointType, typename LineType>
PointType closestPointToLineSegment2D(const PointType &points,
                                      const LineType &line_start_points,
                                      const LineType &line_end_points) {

  auto delta = xt::eval(line_end_points - line_start_points);

  auto pt_x = xt::strided_view(points, {xt::ellipsis(), 0});
  auto pt_y = xt::strided_view(points, {xt::ellipsis(), 1});

  auto dx = xt::strided_view(delta, {xt::ellipsis(), xt::newaxis(), 0});
  auto dy = xt::strided_view(delta, {xt::ellipsis(), xt::newaxis(), 1});

  auto sq_norm = xt::norm_sq(delta, {delta.dimension() - 1});
  auto sq_norm_extended =
      xt::strided_view(sq_norm, {xt::ellipsis(), xt::newaxis()});

  auto st_x =
      xt::strided_view(line_start_points, {xt::ellipsis(), xt::newaxis(), 0});

  auto st_y =
      xt::strided_view(line_start_points, {xt::ellipsis(), xt::newaxis(), 1});

  // clang-format off
  auto u = ((pt_x - st_x) * dx + 
            (pt_y - st_y) * dy) /
        /*-------------------------*/     //->  [ L... P... ]
                sq_norm_extended;


  double eps = 0.000000000001;
  std::vector<size_t> result_shape(points.shape().begin(), points.shape().end());
  result_shape.insert(result_shape.end(), sq_norm.shape().begin(), sq_norm.shape().end());

  static auto toStr = [](auto &&container) {
    std::stringstream ss;
    ss << "[ ";
    for (auto val : container) {
      ss << val << ' ';
    }
    ss << "]";

    return ss.str();
  };

  std::cout << toStr(result_shape);

  auto closest_points = xt::where(xt::abs(xt::broadcast(sq_norm, result_shape)) < eps, 
                          xt::broadcast(line_start_points, result_shape), 
                        xt::where(u <= 0, 
                          xt::broadcast(line_start_points, result_shape),
                        xt::where(u >= 1, xt::broadcast(line_end_points, result_shape), 
                          xt::broadcast(line_start_points, result_shape) + 
                          u * xt::broadcast(delta, result_shape))));
  // clang-format on

  return closest_points;
}

template <typename PointType, typename LineType>
PointType distPointToLineSegment2D(const PointType &points,
                                   const LineType &line_start_points,
                                   const LineType &line_end_points) {
  auto delta = points - closestPointToLineSegment2D(
                            points, line_start_points, line_end_points);

  return xt::norm_l2(delta, {delta.dimension() - 1});
}

} // namespace mppi::geometry
