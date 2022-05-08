// Copyright 2022 FastSense, Samsung Research

#include "mppic/optimizers/xtensor/critics/prefer_forward_critic.hpp"

namespace mppi::critics
{

void PreferForwardCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "prefer_forward_cost_power", 1);
  getParam(weight_, "prefer_forward_cost_weight", 5.0);

  RCLCPP_INFO(
    logger_, "PreferForwardCritic instantiated with %d power and %f weight.", power_, weight_);
}

void PreferForwardCritic::evalScore(models::CriticFunctionData & data)
{
  using namespace xt::placeholders;  // NOLINT

  auto dx = xt::view(data.trajectories, xt::all(), xt::range(1, _), 0) -
    xt::view(data.trajectories, xt::all(), xt::range(_, -1), 0);
  auto dy = xt::view(data.trajectories, xt::all(), xt::range(1, _), 1) -
    xt::view(data.trajectories, xt::all(), xt::range(_, -1), 1);

  auto thetas = xt::atan2(dy, dx);
  auto yaws = xt::view(data.trajectories, xt::all(), xt::range(_, -1), 2);

  auto yaws_local = xt::abs(utils::shortest_angular_distance(thetas, yaws));
  auto forward_translation_reversed = -xt::cos(yaws_local) * xt::hypot(dx, dy);
  auto backward_translation = xt::maximum(forward_translation_reversed, 0);

  data.costs += xt::pow(xt::sum(backward_translation, {1}) * weight_, power_);
}


}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::PreferForwardCritic, mppi::critics::CriticFunction)
