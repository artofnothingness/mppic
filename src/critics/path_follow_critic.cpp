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
    activate_if_path_reached_ratio_less_than_threshold_,
    "activate_if_path_reached_ratio_less_than_threshold", 0.35);

  getParam(offset_from_furthest_, "offset_from_furthest", 6);
  getParam(path_point_count_from_offseted_furthest_, "path_point_count_from_offseted_furthest", 2);

  getParam(power_, "path_follow_cost_power", 2);
  getParam(weight_, "path_follow_cost_weight", 3.0);
}

void PathFollowCritic::score(models::CriticFunctionData & data)
{
  if (!enabled_) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);

  if (utils::pathRatioReached(data) > activate_if_path_reached_ratio_less_than_threshold_) {
    return;
  }

  auto path_points = xt::view(data.path, xt::all(), xt::range(0, 2));
  auto trajectories_last_points_extended = xt::view(
    data.trajectories, xt::all(),
    xt::all(), xt::newaxis(), xt::range(0, 2));

  auto first_considered_point = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, path_points.shape(0) - 1);

  auto last_considered_point = std::min(
    first_considered_point + path_point_count_from_offseted_furthest_, path_points.shape(0) - 1);

  auto considered_path_points = xt::range(first_considered_point, last_considered_point);
  auto path_interval = xt::view(path_points, considered_path_points, xt::all());

  auto distance_to_furthest = xt::norm_l2(trajectories_last_points_extended - path_interval, {2});
  auto mean_distance_to_furthest = xt::mean(distance_to_furthest, {1, 2});

  // data.costs += xt::pow(weight_ * mean_distance_to_furthest, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathFollowCritic,
  mppi::critics::CriticFunction)
