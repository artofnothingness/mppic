// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include <array>
#include <cstdint>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>

#include <tf2/utils.h>

#include "mppic/models/optimizer_settings.hpp"
#include "mppic/models/state.hpp"

namespace mppi
{

class TrajectoryIntegrator
{
public:
  void reset(const models::OptimizerSettings & s)
  {
    xt::noalias(dx) = xt::zeros<double>({s.batch_size, s.time_steps});
    xt::noalias(dy) = xt::zeros<double>({s.batch_size, s.time_steps});
    xt::noalias(yaw_cos) = xt::zeros<double>({s.batch_size, s.time_steps});
    xt::noalias(yaw_sin) = xt::zeros<double>({s.batch_size, s.time_steps});
    xt::noalias(yaw_offseted) = xt::zeros<double>({s.batch_size, s.time_steps});
  }

  void integrate(xt::xtensor<double, 3> & trajectories, const models::OptimizerSettings & optimizer_settings,
                 const models::State & state, bool is_holonomic) {
    using namespace xt::placeholders;  // NOLINT

    auto w = state.getVelocitiesWZ();
    const double initial_yaw = tf2::getYaw(state.pose.pose.orientation);

    auto yaw = xt::view(trajectories, xt::all(), xt::all(), 2);
    yaw = xt::cumsum(w * optimizer_settings.model_dt, 1) + initial_yaw;
    yaw_offseted = yaw;

    xt::view(yaw_offseted, xt::all(), xt::range(1, _)) =
      xt::view(yaw, xt::all(), xt::range(_, -1));

    xt::view(yaw_offseted, xt::all(), 0) = initial_yaw;

    xt::noalias(yaw_cos) = xt::eval(xt::cos(yaw_offseted));
    xt::noalias(yaw_sin) = xt::eval(xt::sin(yaw_offseted));

    auto vx = state.getVelocitiesVX();
    xt::noalias(dx) = vx * yaw_cos;
    xt::noalias(dy) = vx * yaw_sin;

    if (is_holonomic) {
      auto vy = state.getVelocitiesVY();
      dx = dx - vy * yaw_sin;
      dy = dy + vy * yaw_cos;
    }

     auto x = xt::view(trajectories, xt::all(), xt::all(), 0);
     auto y = xt::view(trajectories, xt::all(), xt::all(), 1);

     x = state.pose.pose.position.x + xt::cumsum(dx * optimizer_settings.model_dt, 1);
     y = state.pose.pose.position.y + xt::cumsum(dy * optimizer_settings.model_dt, 1);
  }

private:
xt::xtensor<double, 2> dx;
xt::xtensor<double, 2> dy;

xt::xtensor<double, 2> yaw_cos;
xt::xtensor<double, 2> yaw_sin;
xt::xtensor<double, 2> yaw_offseted;
};

}  // namespace mppi
