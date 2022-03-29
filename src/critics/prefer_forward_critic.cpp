// Copyright 2022 FastSense, Samsung Research

#include "mppic/critics/prefer_forward_critic.hpp"

namespace mppi::critics
{

void PreferForwardCritic::initialize()
{
  auto node = parent_.lock();

  auto getParam = utils::getParamGetter(node, name_);
  getParam(power_, "prefer_forward_cost_power", 1);
  getParam(weight_, "prefer_forward_cost_weight", 10.0);

  RCLCPP_INFO(
    logger_, "PreferForwardCritic instantiated with %d power and %f weight.", power_, weight_);
}

void PreferForwardCritic::score(
  const geometry_msgs::msg::PoseStamped & /* robot_pos */,
  const models::State & /* state */,
  const xt::xtensor<double, 3> & trajectories,
  const xt::xtensor<double, 2> & /* path */, xt::xtensor<double, 1> & costs,
  nav2_core::GoalChecker * /* goal_checker */)
{
  using namespace xt::placeholders;  // NOLINT

  auto dx = xt::view(trajectories, xt::all(), xt::range(1, _), 0) -
    xt::view(trajectories, xt::all(), xt::range(_, -1), 0);
  auto dy = xt::view(trajectories, xt::all(), xt::range(1, _), 1) -
    xt::view(trajectories, xt::all(), xt::range(_, -1), 1);

  auto thetas = xt::atan2(dy, dx);
  auto yaws = xt::view(trajectories, xt::all(), xt::range(_, -1), 2);

  auto yaws_local = xt::abs(utils::shortest_angular_distance(thetas, yaws));
  auto forward_translation_reversed = -xt::cos(yaws_local) * xt::hypot(dx, dy);
  auto backward_translation = xt::maximum(forward_translation_reversed, 0);

  auto cost = xt::eval(xt::sum(backward_translation, {1}));
  auto cost_normalized = cost / xt::amax(cost);
  costs += xt::pow(cost_normalized * weight_, power_);
}


}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::PreferForwardCritic, mppi::critics::CriticFunction)
