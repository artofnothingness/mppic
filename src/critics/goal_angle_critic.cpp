// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/goal_angle_critic.hpp"

namespace mppi::critics
{

void GoalAngleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.0);

  getParam(
    threshold_to_consider_goal_angle_,
    "threshold_to_consider_goal_angle", 0.20);

  RCLCPP_INFO(
    logger_,
    "GoalAngleCritic instantiated with %d power, %f weight, and %f "
    "angular threshold.",
    power_, weight_, threshold_to_consider_goal_angle_);
}

void GoalAngleCritic::score(models::CriticFunctionData & data)
{
  if (!enabled_) {
    return;
  }

  xt::xtensor<double, 1> tensor_pose = {
    static_cast<double>(data.state.pose.pose.position.x),
    static_cast<double>(data.state.pose.pose.position.y)};

  auto path_points = xt::view(data.path, -1, xt::range(0, 2));

  double points_to_goal_dists = xt::norm_l2(tensor_pose - path_points, {0})();

  if (points_to_goal_dists < threshold_to_consider_goal_angle_) {
    auto yaws = xt::view(data.trajectories, xt::all(), xt::all(), 2);
    auto goal_yaw = xt::view(data.path, -1, 2);

    data.costs += xt::pow(
      xt::mean(xt::abs(utils::shortest_angular_distance(yaws, goal_yaw)), {1}) *
      weight_, power_);
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GoalAngleCritic,
  mppi::critics::CriticFunction)
