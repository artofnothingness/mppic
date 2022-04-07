// Copyright 2022 FastSense, Samsung Research
#include "mppic/critics/reference_trajectory_critic.hpp"

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::critics
{

void ReferenceTrajectoryCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(reference_cost_power_, "reference_cost_power", 1);
  getParam(reference_cost_weight_, "reference_cost_weight", 3.0);

  getParam(enable_nearest_path_angle_critic_, "enable_nearest_path_angle_critic", true);
  getParam(nearest_path_angle_offset_, "nearest_path_angle_offset", 6);
  getParam(nearest_path_angle_cost_power_, "nearest_path_angle_cost_power", 1);
  getParam(nearest_path_angle_cost_weight_, "nearest_path_angle_cost_weight", 1.0);

  getParam(enable_nearest_goal_critic_, "enable_nearest_goal_critic", true);
  getParam(nearest_goal_offset_, "nearest_goal_offset", 2);
  getParam(nearest_goal_count_, "nearest_goal_count", 2);
  getParam(nearest_goal_cost_power_, "nearest_goal_cost_power", 1);
  getParam(nearest_goal_cost_weight_, "nearset_goal_cost_weight", 1.0);

  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight."
    "Additional nearest angles critic %d, nearest goal critic %d",
    reference_cost_power_, reference_cost_weight_, enable_nearest_path_angle_critic_,
    enable_nearest_goal_critic_);
}

void ReferenceTrajectoryCritic::score(
  const geometry_msgs::msg::PoseStamped & robot_pose, const models::State & /*state*/,
  const xt::xtensor<double, 3> & trajectories,
  const xt::xtensor<double, 2> & path, xt::xtensor<double, 1> & costs,
  nav2_core::GoalChecker * goal_checker)
{
  if (utils::withinPositionGoalTolerance(goal_checker, robot_pose, path)) {
    return;
  }

  using namespace xt::placeholders;  // NOLINT

  size_t trajectories_count = trajectories.shape(0);
  size_t trajectories_points_count = trajectories.shape(1);
  size_t reference_segments_count = path.shape(0) - 1;

  auto && cost = xt::xtensor<double, 1>::from_shape({trajectories_count});


  // see http://paulbourke.net/geometry/pointlineplane/
  const auto & P3 = trajectories;  // P3 points from which we calculate distance to segments
  auto P1 = xt::view(path, xt::range(_, -1), xt::all());  // segments start points
  auto P2 = xt::view(path, xt::range(1, _), xt::all());  // segments end points

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

  size_t max_s = 0;
  for (size_t t = 0; t < trajectories_count; ++t) {
    double mean_dist = 0;
    for (size_t p = 0; p < trajectories_points_count; ++p) {
      double min_dist = std::numeric_limits<double>::max();
      size_t min_s = 0;
      for (size_t s = 0; s < reference_segments_count; ++s) {
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
        if (dist < min_dist) {
          min_s = s;
          min_dist = dist;
        }
      }
      max_s = std::max(max_s, min_s);
      mean_dist += min_dist;
    }

    cost(t) = mean_dist / trajectories_points_count;
  }

  costs += xt::pow(cost * reference_cost_weight_, reference_cost_power_);

  if (enable_nearest_path_angle_critic_) {
    cost = xt::zeros<double>({trajectories_count});

    auto path_angle_point = std::min(
      reference_segments_count - 1,
      max_s + nearest_path_angle_offset_);

    auto goal_x = xt::view(P2, path_angle_point, 0);
    auto goal_y = xt::view(P2, path_angle_point, 1);
    auto traj_xs = xt::view(P3, xt::all(), xt::all(), 0);
    auto traj_ys = xt::view(P3, xt::all(), xt::all(), 1);
    auto traj_yaws = xt::view(P3, xt::all(), xt::all(), 2);

    auto yaws_between_points = xt::atan2(goal_y - traj_ys, goal_x - traj_xs);

    auto yaws = xt::abs(utils::shortest_angular_distance(traj_yaws, yaws_between_points));
    costs += xt::pow(
      xt::mean(
        yaws,
        {1}) * nearest_path_angle_cost_weight_,
      nearest_path_angle_cost_power_);
  }

  if (enable_nearest_goal_critic_) {
    cost = xt::zeros<double>({trajectories_count});
    size_t last_trajectory_point = trajectories_points_count - 1;
    auto beg = std::min(reference_segments_count - 1, max_s + nearest_goal_offset_);
    auto end = std::min(reference_segments_count, beg + nearest_goal_count_ + 1);

    for (size_t t = 0; t < trajectories_count; ++t) {
      double mean_dist = 0;

      for (size_t i = beg; i < end; ++i) {
        double dx = P2(i, 0) - P3(t, last_trajectory_point, 0);
        double dy = P2(i, 1) - P3(t, last_trajectory_point, 1);
        auto dist = std::hypot(dx, dy);
        mean_dist += dist;
      }
      cost(t) += mean_dist / (end - beg);
    }
    costs += xt::pow(cost * nearest_goal_cost_weight_, nearest_goal_cost_power_);
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ReferenceTrajectoryCritic,
  mppi::critics::CriticFunction)
