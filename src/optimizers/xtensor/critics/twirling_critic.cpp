// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/optimizers/xtensor/critics/twirling_critic.hpp"

namespace mppi::xtensor::critics
{

void TwirlingCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "twirling_cost_power", 1);
  getParam(weight_, "twirling_cost_weight", 10.0);

  RCLCPP_INFO(
    logger_, "TwirlingCritic instantiated with %d power and %f weight.", power_, weight_);
}

void TwirlingCritic::evalScore(models::CriticFunctionData & data)
{
  auto wz = xt::abs(data.state.getVelocitiesWZ());
  data.costs += xt::pow(xt::mean(wz, {1}) * weight_, power_);
}

}  // namespace mppi::xtensor::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::xtensor::critics::TwirlingCritic, mppi::xtensor::critics::CriticFunction)
