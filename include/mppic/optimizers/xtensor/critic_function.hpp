// Copyright 2022 artofnothingness Alexey Budyakov, Samsung Research

#pragma once

#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "mppic/parameters_handler.hpp"
#include "mppic/optimizers/xtensor/models/critic_function_data.hpp"

namespace mppi::xtensor::critics
{
class CriticFunction
{
public:
  CriticFunction() = default;
  virtual ~CriticFunction() = default;

  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * param_handler)
  {
    parent_ = parent;
    logger_ = parent_.lock()->get_logger();
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    parameters_handler_ = param_handler;

    initialize();
  }

  void score(models::CriticFunctionData & data)
  {
    if (data.stop_flag) {
      return;
    }

    evalScore(data);
  }

  virtual void initialize() = 0;

  virtual void evalScore(models::CriticFunctionData & data) = 0;

  std::string getName()
  {
    return name_;
  }

protected:
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  ParametersHandler * parameters_handler_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi::xtensor::critics
