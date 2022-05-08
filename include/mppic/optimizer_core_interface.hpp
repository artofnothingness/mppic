// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include <string>
#include <memory>

// 3rdparty
#include <experimental/mdspan>

// ros
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// msgs
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

// nav2
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/goal_checker.hpp"

#include "mppic/models/optimizer_settings.hpp"
#include "mppic/parameters_handler.hpp"

namespace stdex = std::experimental;

namespace mppi
{
using span2d = stdex::mdspan<double, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>;
using span3d = stdex::mdspan<double, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent,
    stdex::dynamic_extent>>;

class IOptimizerCore
{
public:
  IOptimizerCore() = default;
  virtual ~IOptimizerCore() = default;

  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * dynamic_parameters_handler) = 0;

  virtual geometry_msgs::msg::TwistStamped evalControl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed, const nav_msgs::msg::Path & plan,
    nav2_core::GoalChecker * goal_checker) = 0;

  virtual span3d getGeneratedTrajectories() = 0;
  virtual span2d getOptimizedTrajectory() = 0;


  void setConstraints(double speed_limit, bool percentage);

protected:
  models::OptimizerSettings settings_;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string name_;
  ParametersHandler * parameters_handler_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi
