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
  bool consider_angle_;
  size_t furthest_angle_point_offset_{0};
  unsigned int angle_cost_power_{0};
  double angle_cost_weight_{0};

  bool consider_distance_;
  size_t distance_goal_count_{0};
  size_t distance_offset_{0};
  unsigned int distance_cost_power_{0};
  double distance_cost_weight_{0};

  double distance_to_goal_upper_activation_threshold_{0};
};

}  // namespace mppi::critics
