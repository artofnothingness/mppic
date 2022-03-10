#include "mppic/critics/reference_trajectory_critic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ReferenceTrajectoryCritic<float>, mppi::critics::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ReferenceTrajectoryCritic<double>, mppi::critics::CriticFunction<double>)
