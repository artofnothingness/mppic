// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/path_align_critic.hpp"

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::critics
{

void PathAlignCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 2.0);

  getParam(path_point_step_, "path_point_step", 2);
  getParam(trajectory_point_step_, "trajectory_point_step", 3);

  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
    power_, weight_);
}

void PathAlignCritic::score(CriticData & data)
{
  if (!enabled_ ||
    utils::withinPositionGoalTolerance(data.goal_checker, data.state.pose.pose, data.path))
  {
    return;
  }

  using namespace xt::placeholders;  // NOLINT
  using xt::evaluation_strategy::immediate;

  // see http://paulbourke.net/geometry/pointlineplane/

  // P3 points from which we calculate distance to segments
  const auto & P3_x = data.trajectories.x;
  const auto & P3_y = data.trajectories.y;

  const auto P1_x = xt::view(data.path.x, xt::range(_, -1));  // segments start points
  const auto P1_y = xt::view(data.path.y, xt::range(_, -1));  // segments start points

  const auto P2_x = xt::view(data.path.x, xt::range(1, _));  // segments end points
  const auto P2_y = xt::view(data.path.y, xt::range(1, _));  // segments end points

  const size_t batch_size = P3_x.shape(0);
  const size_t time_steps = P3_x.shape(1);
  const size_t path_segments_count = data.path.x.shape(0) - 1;

  auto && cost = xt::xtensor<float, 1>::from_shape({batch_size});

  const auto P2_P1_dx = P2_x - P1_x;
  const auto P2_P1_dy = P2_y - P1_y;

  const auto && P2_P1_norm_sq = xt::eval(P2_P1_dx * P2_P1_dx + P2_P1_dy * P2_P1_dy);

  auto evaluate_u = [&](size_t t, size_t p, size_t s) {
      return ((P3_x(t, p) - P1_x(s)) * P2_P1_dx(s)) + ((P3_y(t, p) - P1_y(s)) * P2_P1_dy(s)) /
             P2_P1_norm_sq(s);
    };

  const auto segment_short = P2_P1_norm_sq < 1e-3f;
  auto evaluate_dist_sq = [&P3_x, &P3_y](const auto & P,
      size_t t, size_t p) {
      auto dx = P(0) - P3_x(t, p);
      auto dy = P(1) - P3_y(t, p);
      return dx * dx + dy * dy;
    };

  size_t traj_pts_eval = floor(time_steps / trajectory_point_step_);
  size_t max_s = 0;
  for (size_t t = 0; t < batch_size; ++t) {
    float mean_dist = 0;
    for (size_t p = 0; p < time_steps; p += trajectory_point_step_) {
      double min_dist_sq = std::numeric_limits<float>::max();
      size_t min_s = 0;

      for (size_t s = 0; s < path_segments_count; s += path_point_step_) {
        xt::xtensor_fixed<float, xt::xshape<2>> P;
        if (segment_short(s)) {
          P[0] = P1_x(s);
          P[1] = P1_y(s);
        } else if (auto u = evaluate_u(t, p, s); u <= 0) {
          P[0] = P1_x(s);
          P[1] = P1_y(s);
        } else if (u >= 1) {
          P[0] = P2_x(s);
          P[1] = P2_y(s);
        } else {
          P[0] = P1_x(s) + u * P2_P1_dx(s);
          P[1] = P1_y(s) + u * P2_P1_dy(s);
        }
        auto dist = evaluate_dist_sq(P, t, p);
        if (dist < min_dist_sq) {
          min_s = s;
          min_dist_sq = dist;
        }
      }
      max_s = std::max(max_s, min_s);
      mean_dist += std::sqrt(min_dist_sq);
    }

    cost(t) = mean_dist / traj_pts_eval;
  }

  data.furthest_reached_path_point = max_s;
  data.costs += xt::pow(cost * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAlignCritic,
  mppi::critics::CriticFunction)
