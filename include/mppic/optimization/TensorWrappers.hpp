#pragma once

#include <array>
#include <cstdint>

#include <xtensor/xarray.hpp>
#include <xtensor/xstrided_view.hpp>
#include <xtensor/xview.hpp>

#include "mppic/optimization/TensorDataLayouts.hpp"

namespace mppi::optimization {

template<typename T>
struct ControlSequence
{
public:
  xt::xtensor<T, 2> data;
  ControlSequnceIdxes idx;

  void reset(unsigned int time_steps) { data = xt::zeros<T>({ time_steps, idx.dim() }); }
};

template<typename T>
struct State
{
public:
  xt::xtensor<T, 3> data;
  StateIdxes idx;

  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    data = xt::zeros<T>({ batch_size, time_steps, idx.dim() });
  }

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
};


}// namespace mppi::optimization
