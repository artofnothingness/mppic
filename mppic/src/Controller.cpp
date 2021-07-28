#include "mppi/Controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

template class ultra::mppi::Controller<float>;

PLUGINLIB_EXPORT_CLASS(ultra::mppi::Controller<float>, nav2_core::Controller)
