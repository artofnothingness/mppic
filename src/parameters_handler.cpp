// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/parameters_handler.hpp"

namespace mppi
{

ParametersHandler::ParametersHandler(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent)
{
  node_ = parent;
  auto node = node_.lock();
  logger_ = node->get_logger();
}

void ParametersHandler::start()
{
  auto node = node_.lock();
  on_set_param_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParametersHandler::dynamicParamsCallback, this,
      std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
ParametersHandler::dynamicParamsCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock(parameters_change_mutex_);

  for (auto & pre_cb : pre_callbacks_) {
    pre_cb();
  }

  for (auto & param : parameters) {
    const std::string & param_name = param.get_name();

    if (auto callback = get_param_callbacks_.find(param_name);
      callback != get_param_callbacks_.end())
    {
      callback->second(param);
    } else {
      RCLCPP_WARN(logger_, "Parameter %s not found", param_name.c_str());
    }
  }

  for (auto & post_cb : post_callbacks_) {
    post_cb();
  }

  result.successful = true;
  return result;
}

}  // namespace mppi
