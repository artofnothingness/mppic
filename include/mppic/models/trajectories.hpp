// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

namespace mppi::models
{

struct Trajectories
{
  xt::xtensor<float, 2> x;
  xt::xtensor<float, 2> y;
  xt::xtensor<float, 2> yaws;

  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    x = xt::zeros<float>({batch_size, time_steps});
    y = xt::zeros<float>({batch_size, time_steps});
    yaws = xt::zeros<float>({batch_size, time_steps});
  }

  inline auto getLastPoints() const
  {
    return xt::concatenate(
      xtuple(
        xt::view(x, xt::all(), -1, xt::newaxis()),
        xt::view(y, xt::all(), -1, xt::newaxis())), 1);
  }

};

}
// namespace mppi::models
