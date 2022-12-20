// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mppic/critics/path_align_critic.hpp"

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::critics
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void PathAlignCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 1.0);

  getParam(offset_from_furthest_, "offset_from_furthest", 20);
  getParam(trajectory_point_step_, "trajectory_point_step", 5);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.40f);

  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
    power_, weight_);
}

void PathAlignCritic::score(CriticData & data)
{
  if (!enabled_ ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  if (*data.furthest_reached_path_point < offset_from_furthest_) {
    return;
  }

  utils::setPathCostsIfNotSet(data, costmap_ros_);

  const auto & T_x = data.trajectories.x;
  const auto & T_y = data.trajectories.y;

  const auto P_x = xt::view(data.path.x, xt::range(_, -1));  // path points
  const auto P_y = xt::view(data.path.y, xt::range(_, -1));  // path points

  const size_t batch_size = T_x.shape(0);
  const size_t time_steps = T_x.shape(1);
  const size_t traj_pts_eval = floor(time_steps / trajectory_point_step_);
  const size_t path_segments_count = data.path.x.shape(0) - 1;
  auto && cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});

  if (path_segments_count < 1) {
    return;
  }

  for (size_t t = 0; t < batch_size; ++t) {
    float summed_dist = 0;
    for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
      double min_dist_sq = std::numeric_limits<float>::max();
      size_t min_s = 0;

      // Find closest path segment to the trajectory point
      for (size_t s = 0; s < path_segments_count - 1; s++) {
        xt::xtensor_fixed<float, xt::xshape<2>> P;
        float dx = P_x(s) - T_x(t, p);
        float dy = P_y(s) - T_y(t, p);
        float dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          min_s = s;
        }
      }

      // The nearest path point to align to needs to be not in collision, else
      // let the obstacle critic take over in this region due to dynamic obstacles
      if (min_s != 0 && (*data.path_pts_valid)[min_s]) {
        summed_dist += std::sqrt(min_dist_sq);
      }
    }

    cost[t] = summed_dist / traj_pts_eval;
  }

  data.costs += xt::pow(std::move(cost) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAlignCritic,
  mppi::critics::CriticFunction)
