#include "mppic_critics/ReferenceTrajectoryCritic.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::optimization::ReferenceTrajectoryCritic<float>, mppi::optimization::CriticFunction<float>)
PLUGINLIB_EXPORT_CLASS(mppi::optimization::ReferenceTrajectoryCritic<double>, mppi::optimization::CriticFunction<double>)
