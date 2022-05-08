// Copyright 2022 FastSense, Samsung Research

#pragma once

#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "mppic/optimizers/xtensor/models/state.hpp"


namespace mppi::xtensor::models
{

struct CriticFunctionData
{
  const models::State & state;
  const xt::xtensor<double, 3> & trajectories;
  const xt::xtensor<double, 2> & path;
  nav2_core::GoalChecker * goal_checker;

  xt::xtensor<double, 1> & costs;
  bool & stop_flag;
  std::optional<size_t> furthest_reached_path_point;
};

}  // namespace mppi::xtensor::models
