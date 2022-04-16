// Copyright 2022 FastSense, Samsung Research

#pragma once

#include <functional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mppi
{
enum class ParameterType { Dynamic, Static };

class ParametersHandler
{
public:
  using get_param_func_t = void (const rclcpp::Parameter & param);
  using post_callback_t = void ();
  using pre_callback_t = void ();

  ParametersHandler() = default;
  explicit ParametersHandler(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent);

  void start();

  rcl_interfaces::msg::SetParametersResult dynamicParamsCallback(
    std::vector<rclcpp::Parameter> parameters);

  inline auto getParamGetter(const std::string & ns);

  template<typename SettingT, typename ParamT>
  void getParam(
    SettingT & setting, const std::string & name, ParamT default_value,
    ParameterType param_type = ParameterType::Dynamic);

  template<typename T>
  void addPostCallback(T && callback);

  template<typename T>
  void addPreCallback(T && callback);

  template<typename ParamT, typename SettingT, typename NodeT>
  void setParam(SettingT & setting, const std::string & name, NodeT node) const;

  template<typename T>
  void setDynamicParamCallback(T & setting, const std::string & name);

private:
  template<typename T>
  void addDynamicParamCallback(const std::string & name, T && callback);

  template<typename T>
  static auto as(const rclcpp::Parameter & parameter);

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_param_handler_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::unordered_map<std::string, std::function<get_param_func_t>>
  get_param_callbacks_;

  std::vector<std::function<pre_callback_t>> pre_callbacks_;
  std::vector<std::function<post_callback_t>> post_callbacks_;
};

inline auto ParametersHandler::getParamGetter(const std::string & ns)
{
  return [this, ns](
    auto & setting, const std::string & name, auto default_value,
    ParameterType param_type = ParameterType::Dynamic) {
           getParam(
             setting, ns.empty() ? name : ns + "." + name,
             std::move(default_value), param_type);
         };
}


template<typename T>
void ParametersHandler::addDynamicParamCallback(const std::string & name, T && callback)
{
  get_param_callbacks_[name] = callback;
}

template<typename T>
void ParametersHandler::addPostCallback(T && callback)
{
  post_callbacks_.push_back(callback);
}

template<typename T>
void ParametersHandler::addPreCallback(T && callback)
{
  pre_callbacks_.push_back(callback);
}

template<typename SettingT, typename ParamT>
void ParametersHandler::getParam(
  SettingT & setting, const std::string & name,
  ParamT default_value,
  ParameterType param_type)
{
  auto node = node_.lock();

  nav2_util::declare_parameter_if_not_declared(
    node, name, rclcpp::ParameterValue(default_value));

  setParam<ParamT>(setting, name, node);

  if (param_type == ParameterType::Dynamic) {
    setDynamicParamCallback(setting, name);
  }
}

template<typename ParamT, typename SettingT, typename NodeT>
void ParametersHandler::setParam(
  SettingT & setting, const std::string & name, NodeT node) const
{
  ParamT param_in;
  node->get_parameter(name, param_in);
  setting = static_cast<SettingT>(param_in);
}

template<typename T>
void ParametersHandler::setDynamicParamCallback(T & setting, const std::string & name)
{
  if (get_param_callbacks_.find(name) != get_param_callbacks_.end()) {
    return;
  }

  auto callback = [this, &setting, name](const rclcpp::Parameter & param) {
      setting = as<T>(param);
      RCLCPP_INFO(logger_, "Dynamic parameter changed: %s", std::to_string(param).c_str());
    };

  addDynamicParamCallback(name, callback);

  RCLCPP_INFO(logger_, "Dynamic Parameter added %s", name.c_str());
}

template<typename T>
auto ParametersHandler::as(const rclcpp::Parameter & parameter)
{
  if constexpr (std::is_same_v<T, bool>) {
    return parameter.as_bool();
  } else if constexpr (std::is_integral_v<T>) {
    return parameter.as_int();
  } else if constexpr (std::is_floating_point_v<T>) {
    return parameter.as_double();
  } else if constexpr (std::is_same_v<T, std::string>) {
    return parameter.as_string();
  } else if constexpr (std::is_same_v<T, std::vector<int64_t>>) {
    return parameter.as_integer_array();
  } else if constexpr (std::is_same_v<T, std::vector<double>>) {
    return parameter.as_double_array();
  } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
    return parameter.as_string_array();
  }
}

}  // namespace mppi
