// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include "mppic/models/constraints.hpp"
#include <cstddef>

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

  auto getTrajX() {return xt::view(x, xt::all(), xt::all());}

  auto getTrajY() {return xt::view(y, xt::all(), xt::all());}

  auto getTrajW() {return xt::view(yaws, xt::all(), xt::all());}
};

}  // namespace mppi::models
