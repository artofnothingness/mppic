// Copyright 2022 FastSense, Samsung Research
#pragma once


#include "mppic/critic_function.hpp"
#include "mppic/tools/utils.hpp"

namespace mppi::critics
{

class PreferForwardCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics
