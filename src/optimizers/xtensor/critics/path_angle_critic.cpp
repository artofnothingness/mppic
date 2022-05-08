// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/optimizers/xtensor/critics/path_angle_critic.hpp"

namespace mppi::xtensor::critics
{

void PathAngleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "path_angle_cost_power", 1);
  getParam(weight_, "path_angle_cost_weight", 0.5);


  RCLCPP_INFO(
    logger_,
    "PathAngleCritic instantiated with %d power and %f weight.",
    power_, weight_);
}

void PathAngleCritic::evalScore(models::CriticFunctionData & data)
{
  if (utils::withinPositionGoalTolerance(data.goal_checker, data.state.pose, data.path)) {
    return;
  }

  auto goal_x = xt::view(data.path, -1, 0);
  auto goal_y = xt::view(data.path, -1, 1);
  auto traj_xs = xt::view(data.trajectories, xt::all(), xt::all(), 0);
  auto traj_ys = xt::view(data.trajectories, xt::all(), xt::all(), 1);
  auto traj_yaws = xt::view(data.trajectories, xt::all(), xt::all(), 2);

  auto yaws_between_points = xt::atan2(goal_y - traj_ys, goal_x - traj_xs);

  auto yaws = xt::abs(utils::shortest_angular_distance(traj_yaws, yaws_between_points));
  data.costs += xt::pow(xt::mean(yaws, {1}) * weight_, power_);
}

}  // namespace mppi::xtensor::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::xtensor::critics::PathAngleCritic,
  mppi::xtensor::critics::CriticFunction)
