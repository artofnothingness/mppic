// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "nav2_costmap_2d/footprint_collision_checker.hpp"

#include "mppic/critic_function.hpp"
#include "mppic/models/state.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

class ObstaclesCritic : public CriticFunction
{
public:
  void initialize() override;

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  void evalScore(models::CriticFunctionData & data) override;

protected:
  bool inCollision(unsigned char cost) const;
  double scoreCost(unsigned char cost);
  unsigned char maxCost();
  unsigned char costAtPose(double x, double y, double theta);

protected:
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  collision_checker_{nullptr};

  bool consider_footprint_{true};
  double collision_cost_{0};

  unsigned int power_{0};
  double weight_{0};
};

}  // namespace mppi::critics
