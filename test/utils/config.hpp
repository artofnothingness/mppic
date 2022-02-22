#pragma once
#include <rclcpp/rclcpp.hpp>

namespace config {
/**
 * Adds some parameters for the optimizer to a special container.
 *
 * @param params_ container for optimizer's parameters.
 */
void setUpOptimizerParams(
  int iter, int time_steps, double lookahead_dist, std::string motion_model,
  std::vector<rclcpp::Parameter> & params_, std::string node_name = std::string("dummy"))
{
  params_.emplace_back(rclcpp::Parameter(node_name + ".iteration_count", iter));
  params_.emplace_back(rclcpp::Parameter(node_name + ".time_steps", time_steps));
  params_.emplace_back(rclcpp::Parameter(node_name + ".lookahead_dist", lookahead_dist));
  params_.emplace_back(rclcpp::Parameter(node_name + ".motion_model", motion_model));

  std::string critic_scorer_name = node_name + "." + "CriticScorer";
  params_.emplace_back(rclcpp::Parameter(critic_scorer_name + ".critics_type", "float"));
  params_.emplace_back(rclcpp::Parameter(
    critic_scorer_name + ".critics_names",
    std::vector<std::string>{
      "GoalCritic", "GoalAngleCritic", "ReferenceTrajectoryCritic", "ObstaclesCritic"}));
}

} // namespace config
