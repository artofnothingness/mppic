// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#ifndef MPPIC__CRITIC_MANAGER_HPP_
#define MPPIC__CRITIC_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "mppic/parameters_handler.hpp"
#include "mppic/utils.hpp"
#include "mppic/models/critic_function_data.hpp"
#include "mppic/critic_function.hpp"

namespace mppi
{

class CriticManager
{
public:
  CriticManager() = default;

  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>, ParametersHandler *);

  void evalTrajectoriesScores(
    models::CriticFunctionData & data) const;

protected:
  void getParams();
  void loadCritics();
  std::string getFullName(const std::string & name);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;

  ParametersHandler * parameters_handler_;
  std::vector<std::string> critic_names_;
  std::unique_ptr<pluginlib::ClassLoader<critics::CriticFunction>> loader_;
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // MPPIC__CRITIC_MANAGER_HPP_
