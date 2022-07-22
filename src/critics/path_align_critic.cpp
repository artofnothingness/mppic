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
    utils::withinPositionGoalTolerance(data.goal_checker, data.state.pose, data.path))
  {
    return;
  }

  using namespace xt::placeholders;  // NOLINT
 
  // see http://paulbourke.net/geometry/pointlineplane/
  
  // P3 points from which we calculate distance to segments
  const auto & P3x = data.trajectories.x;  
  const auto & P3y = data.trajectories.y;

  auto P1 = xt::view(data.path, xt::range(_, -1), xt::all());  // segments start points
  auto P2 = xt::view(data.path, xt::range(1, _), xt::all());  // segments end points

  size_t trajectories_count = P3x.shape(0);
  size_t trajectories_points_count = P3x.shape(1);
  size_t reference_segments_count = data.path.shape(0) - 1;

  auto cost = xt::xtensor<float, 1>::from_shape({trajectories_count});

  auto P2_P1_diff = P2 - P1;
  auto P2_P1_norm_sq = xt::eval(xt::norm_sq(P2_P1_diff, {1}));

  static auto evaluate_u = [&P1, &P3x, &P3y, &P2_P1_diff, &P2_P1_norm_sq](
    size_t t, size_t p, size_t s) {
      return ((P3x(t, p) - P1(s, 0)) * P2_P1_diff(s, 0) +
             (P3y(t, p) - P1(s, 1)) * P2_P1_diff(s, 1)) /
             P2_P1_norm_sq(s);
    };

  static auto evaluate_dist = [&P3x, &P3y](
    const auto & P, size_t t, size_t p) {
      auto dx = P(0) - P3x(t, p);
      auto dy = P(1) - P3y(t, p);
      return dx * dx + dy * dy;
    };

  size_t max_s = 0, min_s = 0;
  float min_dist2 = 0, mean_dist = 0, dist2 = 0;
  xt::xtensor_fixed<float, xt::xshape<2>> P;
  for (size_t t = 0; t < trajectories_count; ++t) {
    mean_dist = 0;
    for (size_t p = 0; p < trajectories_points_count; p += trajectory_point_step_) {
      min_dist2 = std::numeric_limits<float>::max();
      min_s = 0;
      for (size_t s = 0; s < reference_segments_count; s += path_point_step_) {
        if (P2_P1_norm_sq(s) < 1e-3f) {
          P[0] = P1(s, 0);
          P[1] = P1(s, 1);
        } else if (auto u = evaluate_u(t, p, s); u <= 0) {
          P[0] = P1(s, 0);
          P[1] = P1(s, 1);
        } else if (u >= 1) {
          P[0] = P2(s, 0);
          P[1] = P2(s, 1);
        } else {
          P[0] = P1(s, 0) + u * P2_P1_diff(s, 0);
          P[1] = P1(s, 1) + u * P2_P1_diff(s, 1);
        }
        dist2 = evaluate_dist(P, t, p);
        if (dist2 < min_dist2) {
          min_s = s;
          min_dist2 = dist2;
        }
      }
      max_s = std::max(max_s, min_s);
      mean_dist += std::sqrt(min_dist2);
    }

    cost(t) = mean_dist / trajectories_points_count;
  }

  data.furthest_reached_path_point = max_s;
  data.costs += xt::pow(cost * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAlignCritic,
  mppi::critics::CriticFunction)
