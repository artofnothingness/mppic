// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include <string>
#include <memory>

// ros
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// nav2
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_core/goal_checker.hpp"

// msgs
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "mppic/parameters_handler.hpp"
#include "mppic/models/optimizer_settings.hpp"
#include "mppic/models/state.hpp"

namespace mppi
{

class OptimizerCore
{
public:
  OptimizerCore() = default;
  virtual ~OptimizerCore() = default;

  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * dynamic_parameters_handler) = 0;


  virtual geometry_msgs::msg::TwistStamped evalControl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed, const nav_msgs::msg::Path & plan,
    nav2_core::GoalChecker * goal_checker) = 0;

  virtual xt::xtensor<double, 3> & getGeneratedTrajectories() = 0;

  virtual xt::xtensor<double, 2> evalTrajectoryFromControlSequence(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed) = 0;


  void setSpeedLimit(double speed_limit, bool percentage)
  {
    auto & s = settings_;
    if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
      s.constraints.vx = s.base_constraints.vx;
      s.constraints.vy = s.base_constraints.vy;
      s.constraints.wz = s.base_constraints.wz;
    } else {
      if (percentage) {
        // Speed limit is expressed in % from maximum speed of robot
        double ratio = speed_limit / 100.0;
        s.constraints.vx = s.base_constraints.vx * ratio;
        s.constraints.vy = s.base_constraints.vy * ratio;
        s.constraints.wz = s.base_constraints.wz * ratio;
      } else {
        // Speed limit is expressed in absolute value
        double ratio = speed_limit / s.base_constraints.vx;
        s.constraints.vx = speed_limit;
        s.constraints.vy = s.base_constraints.vx * ratio;
        s.constraints.wz = s.base_constraints.wz * ratio;
      }
    }
  }

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string name_;
  double controller_frequency_{0};

  ParametersHandler * parameters_handler_;
  models::OptimizerSettings settings_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi
