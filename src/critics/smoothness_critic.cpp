// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/smoothness_critic.hpp"

namespace mppi::critics
{

void SmoothnessCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(vx_power_, "vx_cost_power", 1);
  getParam(vx_weight_, "vx_cost_weight", 0.2);

  getParam(vy_power_, "vy_cost_power", 1);
  getParam(vy_weight_, "vy_cost_weight", 0.2);

  getParam(wz_power_, "wz_cost_power", 1);
  getParam(wz_weight_, "wz_cost_weight", 0.2);
}

void SmoothnessCritic::score(models::CriticFunctionData & data)
{
  using namespace xt::placeholders;  // NOLINT

  if (!enabled_) {
    return;
  }

  auto vx = data.state.getControlVelocitiesVX();
  auto dvx = xt::xtensor<double, 2>::from_shape(vx.shape());
  xt::view(dvx, xt::all(), xt::range(1, _)) = xt::abs(
    xt::view(vx, xt::all(), xt::range(1, _)) - xt::view(vx, xt::all(), xt::range(0, -1)));
  xt::view(dvx, xt::all(), 0) = xt::view(vx, xt::all(), 0) - data.state.speed.linear.x;
  auto vx_cost = xt::pow(xt::mean(dvx, {1}) * vx_weight_, vx_power_);
  data.costs += vx_cost;

  auto wz = data.state.getControlVelocitiesWZ();
  auto dwz = xt::xtensor<double, 2>::from_shape(wz.shape());
  xt::view(dwz, xt::all(), xt::range(1, _)) = xt::abs(
    xt::view(wz, xt::all(), xt::range(1, _)) -
    xt::view(wz, xt::all(), xt::range(0, -1)));
  xt::view(dwz, xt::all(), 0) = xt::view(wz, xt::all(), 0) - data.state.speed.angular.z;
  auto wz_cost = xt::pow(xt::mean(dwz, {1}) * wz_weight_, wz_power_);
  data.costs += wz_cost;

  if (data.state.idx.isHolonomic()) {
    auto vy = data.state.getControlVelocitiesVY();
    auto dvy = xt::xtensor<double, 2>::from_shape(vy.shape());
    xt::view(dvy, xt::all(), xt::range(1, _)) = xt::abs(
      xt::view(vy, xt::all(), xt::range(1, _)) -
      xt::view(vy, xt::all(), xt::range(0, -1)));
    xt::view(dvy, xt::all(), 0) = xt::view(vy, xt::all(), 0) - data.state.speed.linear.y;
    auto vy_cost = xt::pow(xt::mean(dvy, {1}) * vy_weight_, vy_power_);
    data.costs += vy_cost;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::SmoothnessCritic,
  mppi::critics::CriticFunction)
