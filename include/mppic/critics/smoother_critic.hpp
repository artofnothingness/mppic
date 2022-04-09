// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/critic_function.hpp"
#include "mppic/models/state.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

class SmootherCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(
    const geometry_msgs::msg::PoseStamped & /*robot_pose*/,
    const models::State & state, const xt::xtensor<double, 3> & trajectories,
    const xt::xtensor<double, 2> & path, xt::xtensor<double, 1> & costs,
    nav2_core::GoalChecker * goal_checker) override;

protected:
  unsigned int vx_power_{0};
  unsigned int vy_power_{0};
  unsigned int wz_power_{0};
  double vx_weight_{0};
  double vy_weight_{0};
  double wz_weight_{0};
};

}  // namespace mppi::critics
