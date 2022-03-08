#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav2_util/node_utils.hpp>

namespace mppi::utils {

template <typename NodeT>
auto getParamGetter(NodeT * node, const std::string & node_name_)
{
  return [=](auto & param, const std::string & param_name, auto default_value) {
    using OutType = std::decay_t<decltype(param)>;
    using InType = std::decay_t<decltype(default_value)>;

    std::string name = node_name_ + '.' + param_name;
    nav2_util::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(default_value));

    InType param_in;
    node->get_parameter(name, param_in);
    param = static_cast<OutType>(param_in);
  };
}

} // namespace mppi::utils
