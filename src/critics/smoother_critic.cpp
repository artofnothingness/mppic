// Copyright 2022 FastSense, Samsung Research
#include "mppic/critics/smoother_critic.hpp"

namespace mppi::critics
{

void SmootherCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(vx_power_, "smoother_vx_cost_power", 1);
  getParam(vy_power_, "smoother_vy_cost_power", 1);
  getParam(wz_power_, "smoother_wz_cost_power", 1);
  getParam(vy_weight_, "smoother_vx_cost_weight", 2.0);
  getParam(vx_weight_, "smoother_vy_cost_weight", 2.0);
  getParam(wz_weight_, "smoother_wz_cost_weight", 2.0);
}

void SmootherCritic::score(
  const geometry_msgs::msg::PoseStamped & /* robot_pose */,
  const models::State & state,
  const xt::xtensor<double, 3> & /* trajectories */,
  const xt::xtensor<double, 2> & /* path */,
  xt::xtensor<double, 1> & costs,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  using namespace xt::placeholders;   // NOLINT
  xt::xtensor<double, 1> cost;

  auto vx = state.getControlVelocitiesVX();
  auto vy = state.getControlVelocitiesVY();
  auto wz = state.getControlVelocitiesWZ();

  auto dvx = xt::view(vx, xt::all(), xt::range(_, -1)) -
    xt::view(vx, xt::all(), xt::range(1, _));

  auto dvy = xt::view(vy, xt::all(), xt::range(_, -1)) -
    xt::view(vy, xt::all(), xt::range(1, _));

  auto dwz = xt::view(wz, xt::all(), xt::range(_, -1)) -
    xt::view(wz, xt::all(), xt::range(1, _));

  auto dvx_cost = xt::pow(xt::mean(dvx, {1}) * vx_weight_, vx_power_);
  auto dvy_cost = xt::pow(xt::mean(dvy, {1}) * vy_weight_, vy_power_);
  auto dwz_cost = xt::pow(xt::mean(dwz, {1}) * wz_weight_, wz_power_);

  costs += dvx_cost + dvy_cost, +dwz_cost;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::SmootherCritic,
  mppi::critics::CriticFunction)
