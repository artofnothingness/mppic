// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"

#include "mppic/critic_function.hpp"
#include "mppic/models/state.hpp"
#include "mppic/tools/utils.hpp"

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
  void score(CriticData & data) override;

protected:
  bool inCollision(unsigned char cost) const;
  double scoreCost(float cost, const size_t traj_len);
  unsigned char maxCost();
  unsigned char costAtPose(double x, double y, double theta);

  /**
    * @brief Find the min cost of the inflation decay function for which the robot MAY be
    * in collision in any orientation
    * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
    * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
    * since some element of the robot could be in collision
    */
  unsigned char findCircumscribedCost(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap);

protected:
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  collision_checker_{nullptr};

  bool consider_footprint_{true};
  float collision_cost_{0};

  unsigned char possibly_inscribed_cost_;

  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics
