// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/critic_function.hpp"
#include "mppic/models/state.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

class ReferenceTrajectoryCritic : public CriticFunction
{
public:
  void initialize() override;

  /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(models::CriticFunctionData & data) override;

protected:
  unsigned int reference_point_step_{0};
  unsigned int trajectory_point_step_{0};

  unsigned int reference_cost_power_{0};
  double reference_cost_weight_{0};
};

}  // namespace mppi::critics
