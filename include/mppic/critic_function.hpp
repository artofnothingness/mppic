// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
 * @class mppi::critics::CollisionCost
 * @brief Utility for storing cost information
 */
struct CollisionCost
{
  float cost{0};
  bool using_footprint{false};
};

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
  /**
    * @brief Checks if cost represents a collision for a path point
    * @param cost Costmap cost
    * @return bool if in collision
    */
  bool inCollision(float cost)
  {
    bool is_tracking_unknown =
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

    switch (static_cast<unsigned char>(cost)) {
      using namespace nav2_costmap_2d; // NOLINT
      case (LETHAL_OBSTACLE):
        return true;
      case (INSCRIBED_INFLATED_OBSTACLE):
        return true;
      case (NO_INFORMATION):
        return is_tracking_unknown ? false : true;
    }

    return false;
  }

  /**
   * @brief  Checks if the path point for driving is valid
   * @param idx Index of path point to evaluate
   * @param data Data struct
   */
  bool isPathPtValid(size_t idx, CriticData & data)
  {
    const auto path_x = data.path.x(idx);
    const auto path_y = data.path.y(idx);
    unsigned int map_x, map_y;
    auto * costmap = costmap_ros_->getCostmap();
    costmap->worldToMap(path_x, path_y, map_x, map_y);

    if (inCollision(static_cast<float>(costmap->getCost(map_x, map_y)))) {
      return false;
    }

    return true;
  }

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
