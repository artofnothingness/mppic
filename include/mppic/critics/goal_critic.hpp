// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/critic_function.hpp"
#include "mppic/models/state.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
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

}  // namespace mppi::critics
