#pragma once

#include <rclcpp/rclcpp.hpp>

namespace mppi::utils {
namespace details {
template <typename T>
T getParam(std::string const & param_name, T default_value, auto * const node)
{
  T param;

  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, rclcpp::ParameterValue(default_value));
  }
  node->get_parameter(param_name, param);

  return param;
}
} // namespace details

auto getParamGetter(auto * const node, const std::string & node_name_)
{
  return [=](auto & param, const std::string & param_name, auto default_value) {
    std::string name = node_name_ + '.' + param_name;
    param =
      static_cast<std::decay_t<decltype(param)>>(details::getParam(name, default_value, node));
  };
}

} // namespace mppi::utils
