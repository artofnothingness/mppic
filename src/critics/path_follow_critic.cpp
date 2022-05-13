// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/path_follow_critic.hpp"

#include <xtensor/xmath.hpp>
#include <xtensor/xsort.hpp>

namespace mppi::critics
{

void PathFollowCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(
    distance_to_goal_upper_activation_threshold_,
    "distance_to_goal_upper_activation_threshold", 0.5);

  getParam(consider_angle_, "consider_angle", true);
  getParam(offset_from_furthest_for_angle_penalize_, "offset_from_furthest_for_angle_penalize", 6);
  getParam(angle_cost_power_, "angle_cost_power", 1);
  getParam(angle_cost_weight_, "angle_cost_weight", 3.0);

  getParam(consider_distance_, "consider_distance", true);
  getParam(offset_from_furthest_for_distance_penalize_, "offset_from_furthest_for_distance_penalize", 6);
  getParam(distance_goal_count_, "distance_goal_count", 2);
  getParam(distance_cost_power_, "distance_cost_power", 2);
  getParam(distance_cost_weight_, "distance_cost_weight", 3.0);
}

void PathFollowCritic::score(models::CriticFunctionData & data)
{
  if(!enabled_) {
    return;
  }

  auto path_points = xt::view(data.path, xt::all(), xt::range(0, 2));

  // get furthest point among trajectories nearest points to path
  size_t furthest_reached_path_point_idx;
  if (data.furthest_reached_path_point) {
    furthest_reached_path_point_idx = *data.furthest_reached_path_point;
  } else {
    furthest_reached_path_point_idx = findPathFurthestPoint(data);
  }


  auto furthest_reached_path_point = xt::view(path_points, furthest_reached_path_point_idx, xt::all());
  auto goal_point = xt::view(path_points, -1, xt::all());

  auto distance_from_goal_to_furthest_point = xt::norm_l2(furthest_reached_path_point - goal_point);

  if (distance_from_goal_to_furthest_point > distance_to_goal_upper_activation_threshold_) {
    return;
  }

  // calculate range of points to consider as local goals
  auto lower_distance_offset = std::min(furthest_reached_path_point_idx + offset_from_furthest_for_distance_penalize, path_points.shape(0) - 1);
  auto upper_distance_offset = std::min(lower_distance_offset + distance_goal_count_, path_points.shape(0) - 1);

  // calculate angle point as local goal
  auto angle_offset = std::min(
    furthest_reached_path_point_idx + offset_from_furthest_for_angle_penalize_, path_points.shape(
      0) - 1);

  // portion of the path which if reached by trajectories then critic returns

  if (consider_distance_ ) {
    auto path_interval = xt::view(path_points, xt::range(lower_distance_offset, upper_distance_offset), xt::all());
    auto trajectories_last_points = xt::view(data.trajectories, xt::all(), -1, xt::newaxis(), xt::range(0, 2));
    auto distance_to_furthest = xt::norm_l2(trajectories_last_points - path_interval, {2});
    auto mean_distance_to_furthest = xt::mean(distance_to_furthest, {1});
    data.costs += xt::pow(distance_cost_weight_ * mean_distance_to_furthest, distance_cost_power_);
  }

  // GoalAngleCritic for local goal
  if (consider_angle_) {
    auto goal_x = xt::view(data.path, angle_offset, 0);
    auto goal_y = xt::view(data.path, angle_offset, 1);
    auto traj_xs = xt::view(data.trajectories, xt::all(), xt::all(), 0);
    auto traj_ys = xt::view(data.trajectories, xt::all(), xt::all(), 1);

    auto traj_yaws = xt::view(data.trajectories, xt::all(), xt::all(), 2);
    auto yaws_between_points = xt::atan2(goal_y - traj_ys, goal_x - traj_xs);
    auto yaws = xt::abs(utils::shortest_angular_distance(traj_yaws, yaws_between_points));

    auto mean_angles_to_furthest = xt::mean(yaws, {1});

    data.costs += xt::pow(angle_cost_weight_ * mean_angles_to_furthest, angle_cost_power_);
  }
}

size_t PathFollowCritic::findPathFurthestPoint(models::CriticFunctionData & data) {
  auto last_points_ext =
    xt::view(data.trajectories, xt::all(), -1, xt::newaxis(), xt::range(0, 2));
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
  return max_id_by_trajectories;
}


}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathFollowCritic,
  mppi::critics::CriticFunction)
