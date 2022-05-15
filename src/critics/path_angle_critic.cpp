// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/path_angle_critic.hpp"

namespace mppi::critics
{

void PathAngleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(offset_from_furthest_, "offset_from_furthest", 6);
  getParam(power_, "path_angle_cost_power", 1);
  getParam(weight_, "path_angle_cost_weight", 0.5);

  getParam(
    deactivate_if_distance_to_goal_less_than_threshold_,
    "deactivate_if_distance_to_goal_less_than_threshold", 0.5);


  RCLCPP_INFO(
    logger_,
    "PathAngleCritic instantiated with %d power and %f weight.",
    power_, weight_);
}

void PathAngleCritic::score(models::CriticFunctionData & data)
{
  if (!enabled_) {
    return;
  }

  if (utils::withinPositionGoalTolerance(data.goal_checker, data.state.pose, data.path)) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  if (deactivate_if_distance_to_goal_less_than_threshold_ < 
      utils::distanceFromFurthestToGoal(data)) {
    return;
  }

  auto path_points = xt::view(data.path, xt::all(), xt::range(0, 2));

  auto angle_offset = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, path_points.shape(
      0) - 1);

  auto goal_x = xt::view(data.path, angle_offset, 0);
  auto goal_y = xt::view(data.path, angle_offset, 1);
  auto traj_xs = xt::view(data.trajectories, xt::all(), xt::all(), 0);
  auto traj_ys = xt::view(data.trajectories, xt::all(), xt::all(), 1);
  auto traj_yaws = xt::view(data.trajectories, xt::all(), xt::all(), 2);

  auto yaws_between_points = xt::atan2(goal_y - traj_ys, goal_x - traj_xs);

  auto yaws = xt::abs(utils::shortest_angular_distance(traj_yaws, yaws_between_points));
  auto mean_yaws = xt::mean(yaws, {1});
  data.costs += xt::pow(mean_yaws * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAngleCritic,
  mppi::critics::CriticFunction)
