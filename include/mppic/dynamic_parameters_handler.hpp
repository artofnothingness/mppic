// Copyright 2022 FastSense, Samsung Research

#pragma once

#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mppi {

class DynamicParametersHandler {
public:
  using callback_t = void(const std::vector<rclcpp::Parameter> &);
  DynamicParametersHandler() = default;
  DynamicParametersHandler(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent) {
  auto node = parent.lock();
  on_set_param_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &DynamicParametersHandler::dynamicParametersCallback, this, std::placeholders::_1));

  }

  template <typename Callable>
  void add_callback(Callable && callback) {
    callbacks.push_back(std::move(callback));
  }

rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  for (auto &callback : callbacks) {
      callback(parameters);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_param_handler_;
  std::vector<std::function<callback_t>> callbacks;
};


}
