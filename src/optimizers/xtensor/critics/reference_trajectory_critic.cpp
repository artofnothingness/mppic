// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/optimizers/xtensor/critics/reference_trajectory_critic.hpp"

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::xtensor::critics
{

void ReferenceTrajectoryCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(reference_cost_power_, "reference_cost_power", 1);
  getParam(reference_cost_weight_, "reference_cost_weight", 3.0);
  getParam(reference_point_step_, "reference_point_step", 2);

  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
    reference_cost_power_, reference_cost_weight_);
}

void ReferenceTrajectoryCritic::evalScore(models::CriticFunctionData & data)
{
  if (utils::withinPositionGoalTolerance(data.goal_checker, data.state.pose, data.path)) {
    return;
  }

  using namespace xt::placeholders;  // NOLINT
  //
  // see http://paulbourke.net/geometry/pointlineplane/
  const auto & P3 = data.trajectories;  // P3 points from which we calculate distance to segments
  auto P1 = xt::view(data.path, xt::range(_, -1), xt::all());  // segments start points
  auto P2 = xt::view(data.path, xt::range(1, _), xt::all());  // segments end points


  size_t trajectories_count = P3.shape(0);
  size_t trajectories_points_count = P3.shape(1);
  size_t reference_segments_count = data.path.shape(0) - 1;

  auto && cost = xt::xtensor<double, 1>::from_shape({trajectories_count});

  xt::xtensor<double, 2> P2_P1_diff = P2 - P1;
  xt::xtensor<double, 1> P2_P1_norm_sq =
    xt::norm_sq(
    P2_P1_diff, {P2_P1_diff.dimension() - 1},
    xt::evaluation_strategy::immediate);

  auto evaluate_u = [&P1, &P3, &P2_P1_diff, &P2_P1_norm_sq](
    size_t t, size_t p, size_t s) -> double {
      return ((P3(t, p, 0) - P1(s, 0)) * (P2_P1_diff(s, 0)) +
             (P3(t, p, 1) - P1(s, 1)) * (P2_P1_diff(s, 1))) /
             P2_P1_norm_sq(s);
    };

  static constexpr double eps = static_cast<double>(1e-3);  // meters
  auto segment_short = P2_P1_norm_sq < eps;
  auto evaluate_dist = [&P3](xt::xtensor_fixed<double, xt::xshape<2>> P,
      size_t t, size_t p) {
      double dx = P(0) - P3(t, p, 0);
      double dy = P(1) - P3(t, p, 1);
      return std::hypot(dx, dy);
    };

  for (size_t t = 0; t < trajectories_count; ++t) {
    double mean_dist = 0;
    for (size_t p = 0; p < trajectories_points_count; ++p) {
      double min_dist = std::numeric_limits<double>::max();
      for (size_t s = 0; s < reference_segments_count; ++reference_point_step_) {
        xt::xtensor_fixed<double, xt::xshape<2>> P;
        if (segment_short(s)) {
          P[0] = P1(s, 0);
          P[1] = P1(s, 1);
        } else if (double u = evaluate_u(t, p, s); u <= 0) {
          P[0] = P1(s, 0);
          P[1] = P1(s, 1);
        } else if (u >= 1) {
          P[0] = P2(s, 0);
          P[1] = P2(s, 1);
        } else {
          P[0] = P1(s, 0) + u * P2_P1_diff(s, 0);
          P[1] = P1(s, 1) + u * P2_P1_diff(s, 1);
        }
        auto dist = evaluate_dist(std::move(P), t, p);
        min_dist = std::min(min_dist, dist);
      }
      mean_dist += min_dist;
    }

    cost(t) = mean_dist / trajectories_points_count;
  }

  data.costs += xt::pow(cost * reference_cost_weight_, reference_cost_power_);
}

}  // namespace mppi::xtensor::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::xtensor::critics::ReferenceTrajectoryCritic,
  mppi::xtensor::critics::CriticFunction)