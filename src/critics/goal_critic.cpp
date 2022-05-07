// Copyright 2022 FastSense, Samsung Research
#include "mppic/critics/goal_critic.hpp"

namespace mppi::critics
{

void GoalCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "goal_cost_power", 1);
  getParam(weight_, "goal_cost_weight", 5.0);
  RCLCPP_INFO(
    logger_, "GoalCritic instantiated with %d power and %f weight.",
    power_, weight_);
}

void GoalCritic::evalScore(models::CriticFunctionData & data)
{
  const auto goal_points = xt::view(data.path, -1, xt::range(0, 2));

  auto trajectories_end =
    xt::view(data.trajectories, xt::all(), -1, xt::range(0, 2));

  auto dim = trajectories_end.dimension() - 1;

  auto && dists_trajectories_end_to_goal =
    xt::norm_l2(std::move(trajectories_end) - goal_points, {dim});

  data.costs += xt::pow(std::move(dists_trajectories_end_to_goal) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalCritic, mppi::critics::CriticFunction)
