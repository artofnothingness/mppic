#include "mppic_critics/GoalCritic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::optimization::GoalCritic<float>, mppi::optimization::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(mppi::optimization::GoalCritic<double>, mppi::optimization::CriticFunction<double>)
