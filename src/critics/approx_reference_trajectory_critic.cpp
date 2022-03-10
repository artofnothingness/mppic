#include "mppic/critics/approx_reference_trajectory_critic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ApproxReferenceTrajectoryCritic<float>,
  mppi::critics::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ApproxReferenceTrajectoryCritic<double>,
  mppi::critics::CriticFunction<double>)
