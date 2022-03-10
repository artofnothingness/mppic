#include "mppic/optimization/scoring/critics/goal_angle_critic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::optimization::GoalAngleCritic<float>, mppi::optimization::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(
  mppi::optimization::GoalAngleCritic<double>, mppi::optimization::CriticFunction<double>)
