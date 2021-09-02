#pragma once

#include "rclcpp/rclcpp.hpp"

namespace mppi::utils {

template <typename... Components> 
auto configure(Components &&...cmps) 
-> void {
  (cmps.on_configure(), ...);
}

template <typename... Components> 
auto activate(Components &&...cmps) 
-> void
{
  (cmps.on_activate(), ...);
}

template <typename... Components> 
auto cleanup(Components &&...cmps) 
-> void
{
  (cmps.on_cleanup(), ...);
}

template <typename... Components> 
auto deactivate(Components &&...cmps) 
-> void
{
  (cmps.on_deactivate(), ...);
}

template <typename N, typename T> 
auto getParam(std::string const &param_name, T default_value, const std::shared_ptr<N> &node) 
-> T
{
  T param;
  node->declare_parameter(param_name, rclcpp::ParameterValue(default_value));
  node->get_parameter(param_name, param);

  return param;
}

} // namespace mppi::utils
