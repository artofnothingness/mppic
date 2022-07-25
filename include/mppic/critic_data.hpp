// Copyright 2022 @artofnothingness Aleksei Budyakov, Samsung Research

#pragma once

#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "mppic/models/state.hpp"
#include "mppic/models/trajectories.hpp"
#include "mppic/models/path.hpp"


namespace mppi
{

struct CriticData
{
  const models::State & state;
  const models::Trajectories & trajectories;
  const models::Path & path;

  xt::xtensor<float, 1> & costs;
  float & model_dt;

  bool fail_flag;
  nav2_core::GoalChecker * goal_checker;
  std::optional<size_t> furthest_reached_path_point;
};

}  // namespace mppi::models
