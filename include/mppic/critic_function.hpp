// Copyright 2022 artofnothingness Alexey Budyakov, Samsung Research
#ifndef MPPIC__CRITIC_FUNCTION_HPP_
#define MPPIC__CRITIC_FUNCTION_HPP_

#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "mppic/tools/parameters_handler.hpp"
#include "mppic/critic_data.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */
class CriticFunction
{
public:
  /**
    * @brief Constructor for mppi::critics::CriticFunction
    */
  CriticFunction() = default;

  /**
    * @brief Destructor for mppi::critics::CriticFunction
    */
  virtual ~CriticFunction() = default;

  /**
    * @brief Configure critic on bringup
    * @param parent WeakPtr to node
    * @param parent_name name of the controller
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & parent_name,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * param_handler)
  {
    parent_ = parent;
    logger_ = parent_.lock()->get_logger();
    name_ = name;
    parent_name_ = parent_name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    parameters_handler_ = param_handler;

    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(enabled_, "enabled", true);

    initialize();
  }

  /**
    * @brief Main function to score trajectory
    * @param data Critic data to use in scoring
    */
  virtual void score(CriticData & data) = 0;

  /**
    * @brief Initialize critic
    */
  virtual void initialize() = 0;

  /**
    * @brief Get name of critic
    */
  std::string getName()
  {
    return name_;
  }

protected:
  bool enabled_;
  std::string name_, parent_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  ParametersHandler * parameters_handler_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi::critics

#endif  // MPPIC__CRITIC_FUNCTION_HPP_
