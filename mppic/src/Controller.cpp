#include "pluginlib/class_list_macros.hpp"
#include <mppic/ControllerImpl.hpp>

template class mppi::Controller<float>;
template class mppi::Controller<double>;

PLUGINLIB_EXPORT_CLASS(mppi::Controller<float>, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(mppi::Controller<double>, nav2_core::Controller)
