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
  using xt::evaluation_strategy::immediate;
 
  // see http://paulbourke.net/geometry/pointlineplane/
  
  // P3 points from which we calculate distance to segments
  const auto & P3_x = data.trajectories.x;  
  const auto & P3_y = data.trajectories.y;

  const auto P1 = xt::view(data.path, xt::range(_, -1), xt::all());  // segments start points
  const auto P2 = xt::view(data.path, xt::range(1, _), xt::all());  // segments end points

  const size_t trajectories_count = P3_x.shape(0);
  const size_t trajectories_points_count = P3_x.shape(1);
  const size_t reference_segments_count = data.path.shape(0) - 1;

  auto cost = xt::xtensor<float, 1>::from_shape({trajectories_count});

  const xt::xtensor<float, 2> P2_P1_diff = P2 - P1;
  const xt::xtensor<float, 1> P2_P1_norm_sq = xt::eval(xt::norm_sq(P2_P1_diff, {1}));

  auto evaluate_u = [&P1, &P3_x, &P3_y, &P2_P1_diff, &P2_P1_norm_sq](
    size_t t, size_t p, size_t s) {
      return ((P3_x(t, p) - P1(s, 0)) * (P2_P1_diff(s, 0)) +
             (P3_y(t, p) - P1(s, 1)) * (P2_P1_diff(s, 1))) /
             P2_P1_norm_sq(s);
    };

  const auto segment_short = P2_P1_norm_sq < 1e-3f;
  auto evaluate_dist = [&P3_x, &P3_y](const auto & P,
      size_t t, size_t p) {
      auto dx = P(0) - P3_x(t, p);
      auto dy = P(1) - P3_y(t, p);
      return std::sqrt(dx * dx + dy * dy);
    };


  const auto dx = xt::view(data.trajectories.x, xt::all(), xt::range(1, _)) -
    xt::view(data.trajectories.x, xt::all(), xt::range(_, -1));
  const auto dy = xt::view(data.trajectories.y, xt::all(), xt::range(1, _)) -
    xt::view(data.trajectories.y, xt::all(), xt::range(_, -1));

  const auto && trajectories_lengths = xt::sum(xt::pow(dx + dy, 2), {1}, immediate);
  const auto accumulated_path_distances = xt::eval(xt::cumsum(P2_P1_norm_sq));

  size_t max_s = 0;
  for (size_t t = 0; t < trajectories_count; ++t) {
    float mean_dist = 0;
    for (size_t p = 0; p < trajectories_points_count; p += trajectory_point_step_) {
      double min_dist = std::numeric_limits<float>::max();
      size_t min_s = 0;
      bool is_path_longer_than_trajectory = false;

      for (size_t s = 0; s < reference_segments_count; s += path_point_step_) {
        xt::xtensor_fixed<float, xt::xshape<2>> P;
        if (segment_short(s)) {
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
        auto dist = evaluate_dist(P, t, p);
        if (dist < min_dist) {
          min_s = s;
          min_dist = dist;
        }

        if (is_path_longer_than_trajectory) {
          break;
        }

        if (accumulated_path_distances(s) > trajectories_lengths(t)) {
          is_path_longer_than_trajectory = true;
        }
      }
      max_s = std::max(max_s, min_s);
      mean_dist += min_dist;
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
