#include "mppic/critics/angle_to_goal_critic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::AngleToGoalCritic<float>, mppi::critics::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(
  mppi::critics::AngleToGoalCritic<double>, mppi::critics::CriticFunction<double>)
