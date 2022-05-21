// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/critic_function.hpp"
#include "mppic/models/state.hpp"
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
   */
  void score(models::CriticFunctionData & data) override;

protected:
  double activate_if_path_reached_ratio_less_than_threshold_{0};
  size_t offset_from_furthest_{0};

  unsigned int power_{0};
  double weight_{0};
};

}  // namespace mppi::critics
