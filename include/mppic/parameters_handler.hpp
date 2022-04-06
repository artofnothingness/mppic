// Copyright 2022 FastSense, Samsung Research

#pragma once

#include <vector>
#include <functional>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

namespace mppi
{

enum class ParameterType
{
  Dynamic,
  Static
};

class ParametersHandler
{
public:
  using get_param_func_t = void (const rclcpp::Parameter & param);
  using post_callback_t = void ();

  ParametersHandler() = default;
  ParametersHandler(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent)
  {
    node_ = parent;
    auto node = node_.lock();
    logger_ = node->get_logger();
  }

  void start()
  {
    auto node = node_.lock();
    on_set_param_handler_ = node->add_on_set_parameters_callback(
      std::bind(
        &ParametersHandler::dynamicParametersCallback, this, std::placeholders::_1));
  }

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters)
  {
    for (auto & param : parameters) {
      const std::string & param_name = param.get_name();

      auto callback = get_param_callbacks_.find(param_name);
      if (callback != get_param_callbacks_.end()) {
        RCLCPP_INFO(logger_, "Parameter %s found", param_name.c_str());
        callback->second(param);
      } else {
        RCLCPP_WARN(logger_, "Parameter %s not found", param_name.c_str());
      }
    }

    for (auto & post_cb : post_callbacks_) {
      post_cb();
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }


  template<typename T>
  void addPostCallback(T && callback)
  {
    post_callbacks_.push_back(std::move(callback));
  }

  auto getParamGetter(std::string ns)
  {
    return [this, ns = std::move(ns)](auto & setting, std::string name, auto default_value,
             ParameterType param_type = ParameterType::Static) {
             getParam(
               setting, ns == "" ? name : ns + "." + name, std::move(
                 default_value), param_type);
           };
  }

  template<typename SettingT, typename ParamT>
  void getParam(
    SettingT & setting, std::string name, ParamT default_value,
    ParameterType param_type = ParameterType::Static)
  {
    auto node = node_.lock();

    nav2_util::declare_parameter_if_not_declared(
      node, name, rclcpp::ParameterValue(default_value));

    setStaticParam<ParamT>(node, setting, name);

    if (param_type == ParameterType::Dynamic) {
      setDynamicParamCallback(setting, name);
    }
  }

  template<typename ParamT, typename SettingT, typename NodeT>
  void setStaticParam(NodeT node, SettingT & setting, std::string name)
  {
    ParamT param_in;
    node->get_parameter(name, param_in);
    setting = static_cast<SettingT>(param_in);
  }

  template<typename T>
  void setDynamicParamCallback(
    T & setting, std::string name)
  {

    auto found = get_param_callbacks_.find(name);
    if (found != get_param_callbacks_.end()) {
      return;
    }

    get_param_callbacks_[name] = [this, &setting, name](const rclcpp::Parameter & param) {
        using setting_t = T;
        if constexpr (std::is_integral_v<setting_t>) {
          auto param_value = param.as_int();
          setting = param_value;
          RCLCPP_INFO(logger_, "Parameter %s set to %ld", name.c_str(), param_value);
        } else if constexpr (std::is_floating_point_v<setting_t>) {
          auto param_value = param.as_double();
          setting = param_value;
          RCLCPP_INFO(logger_, "Parameter %s set to %f", name.c_str(), param_value);
        }
      };

    RCLCPP_INFO(logger_, "Dynamic Parameter added %s", name.c_str());
  }

private:
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_param_handler_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::unordered_map<std::string, std::function<get_param_func_t>> get_param_callbacks_;
  std::vector<std::function<post_callback_t>> post_callbacks_;
};


}
