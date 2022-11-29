// Copyright 2022 @artofnothingness Aleksei Budyakov, Samsung Research

#ifndef MPPIC__CRITIC_DATA_HPP_
#define MPPIC__CRITIC_DATA_HPP_

#include <memory>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "mppic/models/state.hpp"
#include "mppic/models/trajectories.hpp"
#include "mppic/models/path.hpp"
#include "mppic/motion_models.hpp"


namespace mppi
{

/**
 * @struct mppi::CriticData
 * @brief Data to pass to critics for scoring, including state, trajectories, path, costs, and
 * important parameters to share
 */
struct CriticData
{
  const models::State & state;
  const models::Trajectories & trajectories;
  const models::Path & path;

  xt::xtensor<float, 1> & costs;
  float & model_dt;

  bool fail_flag;
  nav2_core::GoalChecker * goal_checker;
  std::shared_ptr<MotionModel> motion_model;
  std::optional<size_t> furthest_reached_path_point;
};

}  // namespace mppi

#endif  // MPPIC__CRITIC_DATA_HPP_
