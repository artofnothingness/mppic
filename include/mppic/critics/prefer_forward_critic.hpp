// Copyright 2022 FastSense, Samsung Research
#pragma once


#include "mppic/critic_function.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

class PreferForwardCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(models::CriticFunctionData & data) override;

protected:
  unsigned int power_{0};
  double weight_{0};
};

}  // namespace mppi::critics
