// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/smoothness_critic.hpp"

namespace mppi::critics
{

void SmoothnessCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(vx_power_, "vx_cost_power", 1);
  getParam(vx_weight_, "vx_cost_weight", 0.1);

  getParam(vy_power_, "vy_cost_power", 1);
  getParam(vy_weight_, "vy_cost_weight", 0.1);

  getParam(wz_power_, "wz_cost_power", 1);
  getParam(wz_weight_, "wz_cost_weight", 0.1);
}

void SmoothnessCritic::score(models::CriticFunctionData & data)
{
  using namespace xt::placeholders;  // NOLINT
  
  if (!enabled_) {
    return;
  }

  auto vx = data.state.getControlVelocitiesVX();
  auto wz = data.state.getControlVelocitiesWZ();

  auto vx_diff = xt::abs(xt::view(vx, xt::all(), xt::range(1, _)) - xt::view(vx, xt::all(), xt::range(0, -1)));
  auto vx_cost = xt::pow(xt::mean(wz, {1}) * vx_weight_, vx_power_);;

  auto wz_diff = xt::abs(xt::view(vx, xt::all(), xt::range(1, _)) - xt::view(vx, xt::all(), xt::range(0, -1)));
  auto wz_cost = xt::pow(xt::mean(wz, {1}) * wz_weight_, wz_power_);;

  data.costs += vx_cost + wz_cost;

  if (data.state.idx.isHolonomic()) {
    auto vy = data.state.getControlVelocitiesVY();
    auto vy_diff = xt::abs(xt::view(vx, xt::all(), xt::range(1, _)) - xt::view(vx, xt::all(), xt::range(0, -1)));
    auto vy_cost = xt::pow(xt::mean(wz, {1}) * vy_weight_, vy_power_);;
    data.costs += vy_cost;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::SmoothnessCritic,
  mppi::critics::CriticFunction)
