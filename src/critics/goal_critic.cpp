#include "mppic/critics/goal_critic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GoalCritic<float>, mppi::critics::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GoalCritic<double>, mppi::critics::CriticFunction<double>)
