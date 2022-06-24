// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include "mppic/critic_function.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

class SmoothnessCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(models::CriticFunctionData & data) override;

protected:
  unsigned int vx_power_{0};
  double vx_weight_{0};

  unsigned int vy_power_{0};
  double vy_weight_{0};

  unsigned int wz_power_{0};
  double wz_weight_{0};
};

}  // namespace mppi::critics
