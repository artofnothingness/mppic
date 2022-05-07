// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include <string>
#include <memory>

// ros
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// nav2
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "mppic/parameters_handler.hpp"

namespace stdex = std::experimental;

namespace mppi
{

class IOptimizerROS
{
public:
  IOptimizerROS() = default;
  virtual ~IOptimizerROS() = default;

  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * dynamic_parameters_handler) = 0;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string name_;
  ParametersHandler * parameters_handler_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi
