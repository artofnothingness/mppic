#include "mppic/optimization/scoring/critics/angle_to_goal_critic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::optimization::AngleToGoalCritic<float>, mppi::optimization::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(
  mppi::optimization::AngleToGoalCritic<double>, mppi::optimization::CriticFunction<double>)
