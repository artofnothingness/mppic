// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include "mppic/critic_function.hpp"
#include "mppic/models/state.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

class PathFollowCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(models::CriticFunctionData & data) override;

protected:
  double max_path_ratio_{0};
  size_t offset_from_furthest_{0};

  unsigned int power_{0};
  double weight_{0};

};

}  // namespace mppi::critics
