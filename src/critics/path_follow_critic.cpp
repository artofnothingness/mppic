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
    max_path_ratio_,
    "max_path_ratio", 0.35);

  getParam(offset_from_furthest_, "offset_from_furthest", 6);

  getParam(power_, "path_follow_cost_power", 2);
  getParam(weight_, "path_follow_cost_weight", 3.0);
}

void PathFollowCritic::score(models::CriticFunctionData & data)
{
  if (!enabled_) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  if (utils::getPathRatioReached(data) > max_path_ratio_) {
    return;
  }

  auto path_points = xt::view(data.path, xt::all(), xt::range(0, 2));
  auto trajectories_last_points = xt::view(
    data.trajectories, xt::all(), -1, xt::range(0, 2));

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, path_points.shape(0) - 1);

  auto offseted_point = xt::view(path_points, offseted_idx, xt::all());
  auto distance_to_furthest = xt::norm_l2(trajectories_last_points - offseted_point, {1});

  data.costs += xt::pow(weight_ * distance_to_furthest, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathFollowCritic,
  mppi::critics::CriticFunction)
