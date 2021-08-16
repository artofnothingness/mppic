#pragma once

#include "rclcpp/rclcpp.hpp"

namespace mppi::utils {

template <typename... Components>
void configure(Components &&...cmps) {
  (cmps.on_configure(), ...);
}

template <typename... Components>
void activate(Components &&...cmps) {
  (cmps.on_activate(), ...);
}

template <typename... Components>
void cleanup(Components &&...cmps) {
  (cmps.on_cleanup(), ...);
}

template <typename... Components>
void deactivate(Components &&...cmps) {
  (cmps.on_deactivate(), ...);
}

template <typename N, typename T>
T getParam(std::string const &param_name, T default_value, const std::shared_ptr<N> &node) {

  T param;
  node->declare_parameter(param_name, rclcpp::ParameterValue(default_value));
  node->get_parameter(param_name, param);

  return param;
}

} // namespace mppi::utils
