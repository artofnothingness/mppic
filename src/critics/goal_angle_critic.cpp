// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/goal_angle_critic.hpp"

namespace mppi::critics
{

void GoalAngleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.0);

  getParam(threshold_to_consider_goal_angle_, "threshold_to_consider_goal_angle", 0.20);

  RCLCPP_INFO(
    logger_,
    "GoalAngleCritic instantiated with %d power, %f weight, and %f "
    "angular threshold.",
    power_, weight_, threshold_to_consider_goal_angle_);
}

void GoalAngleCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  const auto goal_idx = data.path.shape(0) - 1;

  const auto goal_x = data.path(goal_idx, 0);
  const auto goal_y = data.path(goal_idx, 1);

  const auto dx = data.state.pose.pose.position.x - goal_x;
  const auto dy = data.state.pose.pose.position.y - goal_y;

  const auto dist = std::sqrt(dx * dx + dy * dy);

  if (dist < threshold_to_consider_goal_angle_) {
    const auto goal_yaw = data.path(goal_idx, 2);

    data.costs += xt::pow(
      xt::mean(xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw)), {1}) *
      weight_, power_);
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GoalAngleCritic,
  mppi::critics::CriticFunction)
