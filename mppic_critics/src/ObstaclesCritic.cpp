#include "mppic_critics/ObstaclesCritic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::optimization::ObstaclesCritic<float>, mppi::optimization::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(
  mppi::optimization::ObstaclesCritic<double>, mppi::optimization::CriticFunction<double>)
