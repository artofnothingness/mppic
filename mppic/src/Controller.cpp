#include "mppi/Controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

using Controller = ultra::mppi::Controller<float>;

PLUGINLIB_EXPORT_CLASS(Controller, nav2_core::Controller)
