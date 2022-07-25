// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/path_angle_critic.hpp"

#include "math.h"

namespace mppi::critics
{

void PathAngleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(offset_from_furthest_, "offset_from_furthest", 4);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 2.0);

  getParam(
    max_angle_to_furthest_,
    "max_angle_to_furthest", M_PI_2);


  RCLCPP_INFO(
    logger_,
    "PathAngleCritic instantiated with %d power and %f weight.",
    power_, weight_);
}

void PathAngleCritic::score(CriticData & data)
{

  using xt::evaluation_strategy::immediate;
  if (!enabled_) {
    return;
  }

  if (utils::withinPositionGoalTolerance(data.goal_checker, data.state.pose, data.path)) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, data.path.shape(0) - 1);

  const float goal_x = xt::view(data.path, offseted_idx, 0);
  const float goal_y = xt::view(data.path, offseted_idx, 1);

  if (utils::posePointAngle(data.state.pose.pose, goal_x, goal_y) < max_angle_to_furthest_) {
    return;
  }

  const auto yaws_between_points = xt::atan2(goal_y - data.trajectories.y, goal_x - data.trajectories.x);
  const auto yaws = xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points));

  data.costs += xt::pow(xt::mean(yaws, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAngleCritic,
  mppi::critics::CriticFunction)
