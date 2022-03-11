#pragma once

#include <xtensor/xtensor.hpp>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"

#include "mppic/critic_function.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

template <typename T>
class ObstaclesCritic : public CriticFunction<T>
{
public:
  using CriticFunction<T>::costmap_ros_;
  using CriticFunction<T>::costmap_;
  using CriticFunction<T>::parent_;
  using CriticFunction<T>::name_;
  using CriticFunction<T>::logger_;

  void getParams() override
  {
    auto node = parent_.lock();
    auto getParam = utils::getParamGetter(node, name_);
    getParam(consider_footprint_, "consider_footprint", true);
    getParam(power_, "obstacle_cost_power", 1);
    getParam(weight_, "obstacle_cost_weight", 20);
    getParam(inflation_cost_scaling_factor_, "inflation_cost_scaling_factor", 3.0);
    getParam(inflation_radius_, "inflation_radius", 0.75);

    inscribed_radius_ = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
    collision_checker_.setCostmap(costmap_);
    RCLCPP_INFO(
      logger_,
      "ObstaclesCritic instantiated with %d power and %f weight. "
      "It is using %f inflation scale and %f inflation radius to compute distances."
      "Critic will collision check based on %s cost.",
      power_, weight_, inflation_cost_scaling_factor_,
      inflation_radius_, consider_footprint_ ? "footprint" : "circular");
  }

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  virtual void score(
    const geometry_msgs::msg::PoseStamped & /*robot_pose*/, const xt::xtensor<T, 3> & trajectories,
    const xt::xtensor<T, 2> & /*path*/, xt::xtensor<T, 1> & costs) override
  {
    constexpr T collision_cost_value = std::numeric_limits<T>::max() / 2.0;

    enum TrajectoryState : uint8_t { Collision, Inflated, Free };

    for (size_t i = 0; i < trajectories.shape()[0]; ++i) {
      double min_dist_to_obstacle = std::numeric_limits<double>::max();
      TrajectoryState state = Free;

      for (size_t j = 0; j < trajectories.shape()[1]; ++j) {
        auto point = xt::view(trajectories, i, j, xt::all());
        auto cost = costAtPose(point);
        if (!isFree(cost)) {
          if (inCollision(cost)) {
            state = Collision;
            break;
          }
          state = Inflated;
          min_dist_to_obstacle = std::min(toDist(cost), min_dist_to_obstacle);
        }
      }

      if (state == Inflated) {
        costs[i] += scoreDistance(min_dist_to_obstacle);
      } else if (state == Collision) {
        costs[i] += collision_cost_value;
      }
    }
  }

protected:
  unsigned char costAtPose(const auto & point)
  {
    unsigned char cost;

    if (consider_footprint_) {
      cost = static_cast<unsigned char>(collision_checker_.footprintCostAtPose(
        point(0), point(1), point(2), costmap_ros_->getRobotFootprint()));
    } else {
      cost = static_cast<unsigned char>(collision_checker_.pointCost(point(0), point(1)));
    }

    return cost;
  }

  bool inCollision(unsigned char cost) const
  {
    if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
      return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
             cost != nav2_costmap_2d::NO_INFORMATION;
    }

    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  bool isFree(unsigned char cost) const { return cost == nav2_costmap_2d::FREE_SPACE; }

  double toDist(auto cost)
  {
    return (-1.0 / inflation_cost_scaling_factor_) *
             std::log(
               static_cast<double>(cost) /
               (static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) - 1.0)) +
           inscribed_radius_;
  }

  T scoreDistance(double min_dist)
  {
    return static_cast<T>(pow((1.01 * inflation_radius_ - min_dist) * weight_, power_));
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision_checker_{
    nullptr};

  bool consider_footprint_{true};
  double inflation_cost_scaling_factor_{0};
  double inscribed_radius_{0};
  double inflation_radius_{0};
  unsigned int power_{0};
  double weight_{0};
};

} // namespace mppi::critics
