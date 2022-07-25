// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/goal_critic.hpp"

namespace mppi::critics
{

void GoalCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0);
  RCLCPP_INFO(
    logger_, "GoalCritic instantiated with %d power and %f weight.",
    power_, weight_);
}

void GoalCritic::score(CriticData & data)
{

  if (!enabled_) {
    return;
  }

  const auto goal_idx = data.path.shape(0) - 1;

  const auto goal_x = data.path(goal_idx, 0);
  const auto goal_y = data.path(goal_idx, 1);

  const auto last_x = xt::view(data.trajectories.x, xt::all(), -1);
  const auto last_y = xt::view(data.trajectories.y, xt::all(), -1);

  auto dists = xt::sqrt(xt::pow(last_x - goal_x, 2) + 
                        xt::pow(last_y - goal_y, 2));

  data.costs += xt::pow(std::move(dists) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalCritic, mppi::critics::CriticFunction)
