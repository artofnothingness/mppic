// Copyright 2022 FastSense, Samsung Research
#pragma once


#include "mppic/optimizers/xtensor/critic_function.hpp"
#include "mppic/utils.hpp"

namespace mppi::xtensor::critics
{

class PreferForwardCritic : public CriticFunction
{
public:
  void initialize() override;

  void evalScore(models::CriticFunctionData & data) override;

protected:
  unsigned int power_{0};
  double weight_{0};
};

}  // namespace mppi::xtensor::critics
