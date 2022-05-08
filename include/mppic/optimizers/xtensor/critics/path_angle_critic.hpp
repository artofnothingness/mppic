// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/optimizers/xtensor/critic_function.hpp"
#include "mppic/optimizers/xtensor/models/state.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

class PathAngleCritic : public CriticFunction
{
public:
  void initialize() override;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void evalScore(models::CriticFunctionData & data) override;

protected:
  unsigned int power_{0};
  double weight_{0};
};

}  // namespace mppi::critics
