// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/critic_function.hpp"
#include "mppic/models/state.hpp"
#include "mppic/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for enforcing feasible constraints
 */
class ConstraintCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float min_vel_;
  float max_vel_;
};

}  // namespace mppi::critics
