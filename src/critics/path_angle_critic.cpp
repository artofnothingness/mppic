// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/path_angle_critic.hpp"

#include "math.h"

namespace mppi::critics
{

void PathAngleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(offset_from_furthest_, "offset_from_furthest", 6);
  getParam(power_, "path_angle_cost_power", 1);
  getParam(weight_, "path_angle_cost_weight", 0.5);

  getParam(
    activate_if_angle_to_furthest_more_than_threshold_,
    "activate_if_angle_to_furthest_more_than_threshold", M_PI_2);

  getParam(
    activate_if_path_reached_ratio_less_than_threshold_,
    "activate_if_path_reached_ratio_less_than_threshold", 0.35);


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

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, data.path.shape(0) - 1);

  double goal_x = xt::view(data.path, offseted_idx, 0);
  double goal_y = xt::view(data.path, offseted_idx, 1);

  bool angle_to_furthest_more_than_threshold = utils::pose_point_angle(
    data.state.pose.pose, goal_x,
    goal_y) >
    activate_if_angle_to_furthest_more_than_threshold_;
  bool path_reached_ratio_less_than_threshold = utils::pathRatioReached(data) >
    activate_if_path_reached_ratio_less_than_threshold_;

  if (!angle_to_furthest_more_than_threshold && path_reached_ratio_less_than_threshold) {
    return;
  }

  auto traj_xs = xt::view(data.trajectories, xt::all(), xt::all(), 0);
  auto traj_ys = xt::view(data.trajectories, xt::all(), xt::all(), 1);
  auto yaws_between_points = xt::atan2(goal_y - traj_ys, goal_x - traj_xs);

  auto traj_yaws = xt::view(data.trajectories, xt::all(), xt::all(), 2);
  auto yaws = xt::abs(utils::shortest_angular_distance(traj_yaws, yaws_between_points));
  auto mean_yaws = xt::mean(yaws, {1});

  data.costs += xt::pow(mean_yaws * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAngleCritic,
  mppi::critics::CriticFunction)
