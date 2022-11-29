// Copyright 2022 FastSense, Samsung Research
#pragma once


#include "mppic/critic_function.hpp"
#include "mppic/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for preferring forward motion
 */
class PreferForwardCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float threshold_to_consider_{0};
};

}  // namespace mppi::critics
