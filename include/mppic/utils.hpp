// Copyright 2022 FastSense, Samsung Research
#ifndef MPPIC__UTILS_HPP_
#define MPPIC__UTILS_HPP_

#include <algorithm>
#include <chrono>
#include <string>

#include <xtensor/xarray.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xview.hpp>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mppic/models/control_sequence.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mppi::utils
{

template<typename NodeT>
auto getParamGetter(NodeT node, const std::string & name)
{
  return [ = ](auto & param, const std::string & param_name, auto default_value) {
           using OutType = std::decay_t<decltype(param)>;
           using InType = std::decay_t<decltype(default_value)>;

           std::string full_name;

           if (name != "") {
             full_name = name + '.' + param_name;
           } else {
             full_name = param_name;
           }

           nav2_util::declare_parameter_if_not_declared(
             node, full_name, rclcpp::ParameterValue(default_value));

           InType param_in;
           node->get_parameter(full_name, param_in);
           param = static_cast<OutType>(param_in);
         };
}


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

}  // namespace mppi::utils

#endif  // MPPIC__UTILS_HPP_
