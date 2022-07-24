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
    "max_path_ratio", 0.35f);

  getParam(offset_from_furthest_, "offset_from_furthest", 6);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 2.0);
}

void PathFollowCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  if (utils::getPathRatioReached(data) > max_path_ratio_) {
    return;
  }

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, data.path.shape(0) - 1);

  auto path_x = xt::view(data.path, offseted_idx, 0);
  auto path_y = xt::view(data.path, offseted_idx, 1);

  auto last_x = xt::view(data.trajectories.x, xt::all(), -1);
  auto last_y = xt::view(data.trajectories.y, xt::all(), -1);

  auto dists = xt::sqrt(xt::pow(std::move(last_x) - std::move(path_x) , 2) + 
                        xt::pow(std::move(last_y) - std::move(path_y), 2));

  data.costs += xt::pow(weight_ * std::move(dists), power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathFollowCritic,
  mppi::critics::CriticFunction)
