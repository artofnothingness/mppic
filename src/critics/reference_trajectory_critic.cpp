#include "mppic/critics/reference_trajectory_critic.hpp"

namespace mppi::critics
{

void ReferenceTrajectoryCritic::initialize()
{
  auto node = parent_.lock();
  auto getParam = utils::getParamGetter(node, name_);
  getParam(power_, "reference_cost_power", 1);
  getParam(weight_, "reference_cost_weight", 15);
  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight.", power_, weight_);
}

void ReferenceTrajectoryCritic::score(
  const geometry_msgs::msg::PoseStamped & /*robot_pose*/,
  const xt::xtensor<double, 3> & trajectories,
  const xt::xtensor<double, 2> & path, xt::xtensor<double, 1> & costs)

{
  using xt::evaluation_strategy::immediate;

  xt::xtensor<double, 3> dists_path_to_trajectories =
    utils::distPointsToLineSegments2D(path, trajectories);

  xt::xtensor<double, 1> cost =
    xt::mean(xt::amin(std::move(dists_path_to_trajectories), 1, immediate), 1, immediate);

  costs += xt::pow(std::move(cost) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ReferenceTrajectoryCritic, mppi::critics::CriticFunction)
