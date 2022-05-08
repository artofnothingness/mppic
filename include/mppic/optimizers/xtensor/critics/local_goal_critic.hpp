// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/optimizers/xtensor/critic_function.hpp"
#include "mppic/optimizers/xtensor/models/state.hpp"
#include "mppic/utils.hpp"

namespace mppi::xtensor::critics
{

class LocalGoalCritic : public CriticFunction
{
public:
  void initialize() override;

  void evalScore(models::CriticFunctionData & data) override;

protected:

  bool consider_angle_;
  size_t angle_offset_{0};
  unsigned int angle_cost_power_{0};
  double angle_cost_weight_{0};

  bool consider_distance_;
  size_t distance_goal_count_{0};
  size_t distance_offset_{0};
  unsigned int distance_cost_power_{0};
  double distance_cost_weight_{0};

  double stop_usage_path_reached_ratio_{0};
};

}  // namespace mppi::xtensor::critics
