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

#include "mppic/tools/parameters_handler.hpp"
#include "mppic/tools/utils.hpp"
#include "mppic/critic_data.hpp"
#include "mppic/critic_function.hpp"

namespace mppi
{

/**
 * @class mppi::CriticManager
 * @brief Manager of objective function plugins for scoring trajectories
 */
class CriticManager
{
public:
  /**
    * @brief Constructor for mppi::CriticManager
    */
  CriticManager() = default;

  /**
    * @brief Configure critic manager on bringup and load plugins
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>, ParametersHandler *);

  /**
    * @brief Score trajectories by the set of loaded critic functions
    * @param CriticData Struct of necessary information to pass to the critic functions
    */
  void evalTrajectoriesScores(CriticData & data) const;

protected:
  /**
    * @brief Get parameters (critics to load)
    */
  void getParams();

  /**
    * @brief Load the critic plugins
    */
  void loadCritics();

  /**
    * @brief Get full-name namespaced critic IDs
    */
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
