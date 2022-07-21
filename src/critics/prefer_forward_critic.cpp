// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#include "mppic/critics/prefer_forward_critic.hpp"

namespace mppi::critics
{

void PreferForwardCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.0);

  RCLCPP_INFO(
    logger_, "PreferForwardCritic instantiated with %d power and %f weight.", power_, weight_);
}

void PreferForwardCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  auto backward_motion = xt::maximum(-data.state.vx, 0);
  data.costs += xt::pow(xt::sum(backward_motion * data.model_dt, {1}, xt::evaluation_strategy::immediate) * weight_, power_);
}
}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PreferForwardCritic,
  mppi::critics::CriticFunction)
