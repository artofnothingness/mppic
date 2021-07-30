#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ultra::mppi::utils {

using ManagedNode = rclcpp_lifecycle::LifecycleNode;

template<typename T>
void getParam(std::string const& param_name, T default_value, 
              ManagedNode::SharedPtr node, T &param) {

  node->declare_parameter(param_name, rclcpp::ParameterValue(default_value));
  node->get_parameter(param_name, param);
}

}

