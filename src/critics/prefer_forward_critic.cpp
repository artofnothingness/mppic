// Copyright 2022 FastSense, Samsung Research

#include "mppic/critics/prefer_forward_critic.hpp"

namespace mppi::critics
{

void PreferForwardCritic::initialize()
{
  auto node = parent_.lock();

  auto getParam = utils::getParamGetter(node, name_);
  getParam(power_, "prefer_forward_cost_power", 1);
  getParam(weight_, "prefer_forward_cost_weight", 20.0);

  RCLCPP_INFO(
    logger_, "PreferForwardCritic instantiated with %d power and %f weight.", power_, weight_);
}

void PreferForwardCritic::score(
  const geometry_msgs::msg::PoseStamped & /* robot_pos */, const xt::xtensor<double, 3> & trajectories,
  const xt::xtensor<double, 2> & /* path */, xt::xtensor<double, 1> & costs,
  nav2_core::GoalChecker * /* goal_checker */)
{
  using namespace xt::placeholders;

  xt::xtensor<T, 2> x_diff = xt::view(trajectories, xt::all(), xt::range(1, _), 0) -
              xt::view(trajectories, xt::all(), xt::range(_, -1), 0);
  xt::xtensor<T, 2> y_diff = xt::view(trajectories, xt::all(), xt::range(1, _), 1) -
              xt::view(trajectories, xt::all(), xt::range(_, -1), 1);

  xt::xtensor<T, 2> yaws = xt::view(trajectories, xt::all(), xt::range(_, -1), 2);
  xt::xtensor<T, 2> thetas = xt::atan2(y_diff, x_diff) - yaws - M_PI;
  xt::xtensor<T, 2> v_negative = xt::cos(thetas) * xt::sqrt(x_diff * x_diff + y_diff * y_diff);
  v_negative = xt::maximum(v_negative, 0);

  costs += xt::pow(xt::mean(v_negative, {1}) * weight_, power_);
}


}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::PreferForwardCritic, mppi::critics::CriticFunction)


