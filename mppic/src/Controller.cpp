#include "mppi/impl/Controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

template class mppi::Controller<float>;

PLUGINLIB_EXPORT_CLASS(mppi::Controller<float>, nav2_core::Controller)
