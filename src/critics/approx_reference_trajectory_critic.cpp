#include "mppic/optimization/scoring/critics/approx_reference_trajectory_critic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::optimization::ApproxReferenceTrajectoryCritic<float>,
  mppi::optimization::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(
  mppi::optimization::ApproxReferenceTrajectoryCritic<double>,
  mppi::optimization::CriticFunction<double>)
