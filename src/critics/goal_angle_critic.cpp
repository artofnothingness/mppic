#include "mppic/critics/goal_angle_critic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GoalAngleCritic<float>, mppi::critics::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GoalAngleCritic<double>, mppi::critics::CriticFunction<double>)
