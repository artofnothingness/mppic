#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ultra::mppi::utils {

using ManagedNode = rclcpp_lifecycle::LifecycleNode;

void getParam(std::string const& param_name, auto default_value, 
              ManagedNode::SharedPtr node, auto &param) {

  node->declare_parameter(param_name, rclcpp::ParameterValue(default_value));
  node->get_parameter(param_name, param);
}

}

