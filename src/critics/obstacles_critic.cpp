#include "mppic/critics/obstacles_critic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ObstaclesCritic<float>, mppi::critics::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ObstaclesCritic<double>, mppi::critics::CriticFunction<double>)
