// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#include "mppic/critics/prefer_forward_critic.hpp"

namespace mppi::critics
{

void PreferForwardCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.0);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.40f);

  RCLCPP_INFO(
    logger_, "PreferForwardCritic instantiated with %d power and %f weight.", power_, weight_);
}

void PreferForwardCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_) {
    return;
  }

  if (utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path)) {
    return;
  }

  auto backward_motion = xt::maximum(-data.state.vx, 0);
  data.costs += xt::pow(
    xt::sum(
      std::move(
        backward_motion) * data.model_dt, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PreferForwardCritic,
  mppi::critics::CriticFunction)
