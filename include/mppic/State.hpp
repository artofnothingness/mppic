#pragma once

#include <array>

#include <xtensor/xarray.hpp>
#include <xtensor/xstrided_view.hpp>
#include <xtensor/xview.hpp>

#include "mppic/MotionModel.hpp"

namespace mppi::optimization {
template<typename T>
class State
{
public:
  xt::xtensor<T, 3> data;

  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    data = xt::zeros<T>({ batch_size, time_steps, vec_dim_ });
  }
  unsigned int getVectorDimension() const { return vec_dim_; }

  void setMotionModel(MotionModel motion_model)
  {
    if (isHolonomic(motion_model)) {
      idx.vx = 0;
      idx.vy = 1;
      idx.wz = 2;
      idx.cvx = 3;
      idx.cvy = 4;
      idx.cwz = 5;
      idx.dt = 6;
      vec_dim_ = 7;
    } else {
      idx.vx = 0;
      idx.wz = 1;
      idx.cvx = 2;
      idx.cwz = 3;
      idx.dt = 4;
      idx.vy = 0;
      idx.cvy = 0;
      vec_dim_ = 5;
    }

    idx.velocities_range[0] = idx.vx;
    idx.velocities_range[1] = idx.cvx;
    idx.controls_range[0] = idx.cvx;
    idx.controls_range[1] = idx.dt;
  }

  struct Idx
  {
    uint8_t vx{ 0 };
    uint8_t vy{ 0 };
    uint8_t wz{ 0 };
    uint8_t cvx{ 0 };
    uint8_t cvy{ 0 };
    uint8_t cwz{ 0 };
    uint8_t dt{ 0 };
    std::array<uint8_t, 2> velocities_range{ 0, 0 };
    std::array<uint8_t, 2> controls_range{ 0, 0 };
  } idx;

  auto getControls() const;
  auto getControls();
  auto getVelocities();
  auto getVelocities() const;
  auto getVelocitiesVX() const;
  auto getVelocitiesVX();
  auto getVelocitiesVY() const;
  auto getVelocitiesVY();
  auto getVelocitiesWZ() const;
  auto getVelocitiesWZ();
  auto getControlVelocitiesVX() const;
  auto getControlVelocitiesVX();
  auto getControlVelocitiesVY() const;
  auto getControlVelocitiesVY();
  auto getControlVelocitiesWZ() const;
  auto getControlVelocitiesWZ();
  auto getTimeIntervals();
  auto getTimeIntervals() const;

private:
  unsigned int vec_dim_{ 0 };
};

}// namespace mppi::optimization
