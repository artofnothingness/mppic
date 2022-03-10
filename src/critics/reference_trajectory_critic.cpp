#include "mppic/optimization/scoring/critics/reference_trajectory_critic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::optimization::ReferenceTrajectoryCritic<float>, mppi::optimization::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(
  mppi::optimization::ReferenceTrajectoryCritic<double>, mppi::optimization::CriticFunction<double>)
