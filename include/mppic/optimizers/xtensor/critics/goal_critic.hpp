// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/optimizers/xtensor/critic_function.hpp"
#include "mppic/optimizers/xtensor/models/state.hpp"
#include "mppic/utils.hpp"

namespace mppi::xtensor::critics
{

class GoalCritic : public CriticFunction
{
public:
  void initialize() override;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void evalScore(models::CriticFunctionData & data) override;

protected:
  unsigned int power_{0};
  double weight_{0};
};

}  // namespace mppi::xtensor::critics
