#include "mppic/optimization/scoring/critics/ApproxReferenceTrajectoryCritic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::optimization::ApproxReferenceTrajectoryCritic<float>,
  mppi::optimization::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(
  mppi::optimization::ApproxReferenceTrajectoryCritic<double>,
  mppi::optimization::CriticFunction<double>)
