// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/critic_function.hpp"
#include "mppic/models/state.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

class ReferenceTrajectoryCritic : public CriticFunction
{
public:
  void initialize() override;

  /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(
    const geometry_msgs::msg::PoseStamped & /*robot_pose*/,
    const models::State & state, const xt::xtensor<double, 3> & trajectories,
    const xt::xtensor<double, 2> & path, xt::xtensor<double, 1> & costs,
    nav2_core::GoalChecker * goal_checker) override;

protected:
  unsigned int reference_cost_power_{0};
  double reference_cost_weight_{0};

  bool enable_nearest_path_angle_critic_;
  size_t nearest_path_angle_offset_{0};
  unsigned int nearest_path_angle_cost_power_{0};
  double nearest_path_angle_cost_weight_{0};

  bool enable_nearest_goal_critic_;
  size_t nearest_goal_offset_;
  size_t nearest_goal_count_;
  unsigned int nearest_goal_cost_power_{0};
  double nearest_goal_cost_weight_{0};


};

}  // namespace mppi::critics
