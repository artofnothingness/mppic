// Copyright 2022 FastSense, Samsung Research
#include "mppic/critics/local_goal_critic.hpp"

#include <xtensor/xmath.hpp>
#include <xtensor/xsort.hpp>

namespace mppi::critics
{

void LocalGoalCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(goal_count_, "goal_count", 2);
  getParam(goal_offset_, "goal_offset", 2);

  getParam(angle_offset_, "angle_offset", 6);
  getParam(angle_cost_power_, "angle_cost_power", 1);
  getParam(angle_cost_weight_, "angle_cost_weight", 1.0);

  getParam(distance_cost_power_, "distance_cost_power", 1);
  getParam(distance_cost_weight_, "distance_cost_weight", 1.0);

  getParam(stop_usage_path_reached_ratio_, "stop_usage_path_reached_ratio", 0.7);
}

void LocalGoalCritic::evalScore(models::CriticFunctionData & data)
{
  auto last_points_ext = xt::view(data.trajectories, xt::all(), -1, xt::newaxis(), xt::range(0, 2));
  auto path_points = xt::view(
      data.path, xt::all(), xt::range(0, 2));

  auto distances = xt::norm_l2(last_points_ext - path_points, {2});

  size_t max_id_by_trajectories = 0;
  double min_distance_by_path = std::numeric_limits<double>::max();
  for (size_t i = 0; i < distances.shape(0); i++) {
    size_t min_id_by_path = 0;
    for (size_t j = 0; j < distances.shape(1); j++) {
      if (min_distance_by_path < distances(i, j)) {
        min_distance_by_path = distances(i, j);
        min_id_by_path = j;
      }
    }
    max_id_by_trajectories = std::max(max_id_by_trajectories, min_id_by_path);
  }

  auto offset = std::min(max_id_by_trajectories + goal_offset_, path_points.shape(0) - 1);
  auto upper_distance_offset = std::min(offset + goal_count_, path_points.shape(0) - 1);

  auto angle_upper_offset  = std::min(max_id_by_trajectories + angle_offset_, path_points.shape(0) - 1);

  size_t threshold_idx = static_cast<size_t>(
      static_cast<double>(data.path.shape(0) - 1) * stop_usage_path_reached_ratio_);

  if (upper_distance_offset < threshold_idx) {
    auto path_interval = xt::view(path_points, xt::range(offset, upper_distance_offset), xt::all());
    auto trajectories_points = xt::view(data.trajectories, xt::all(), xt::all(), xt::newaxis(), xt::range(0, 2));
    auto distance_to_furthest = xt::norm_l2(trajectories_points - path_interval, {2});
    auto mean_distance_to_furthest = xt::mean(distance_to_furthest, {1, 2});
    data.costs += xt::pow(distance_cost_weight_ * mean_distance_to_furthest, distance_cost_power_);
  }

  if (angle_upper_offset < threshold_idx) {
    auto goal_x = xt::view(data.path, angle_upper_offset, 0);
    auto goal_y = xt::view(data.path, angle_upper_offset, 1);

    auto traj_xs = xt::view(data.trajectories, xt::all(), xt::all(), 0);
    auto traj_ys = xt::view(data.trajectories, xt::all(), xt::all(), 1);

    auto traj_yaws = xt::view(data.trajectories, xt::all(), xt::all(), 2);

    auto yaws_between_points = xt::atan2(goal_y - traj_ys, goal_x - traj_xs);
    auto yaws = xt::abs(utils::shortest_angular_distance(traj_yaws, yaws_between_points));

    auto mean_angles_to_furthest = xt::mean(yaws, {1});
    
    data.costs += xt::pow(angle_cost_weight_ * mean_angles_to_furthest, angle_cost_power_);
  }
}


}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::LocalGoalCritic,
  mppi::critics::CriticFunction)
