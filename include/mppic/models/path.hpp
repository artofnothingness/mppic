// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include <xtensor/xtensor.hpp>

namespace mppi::models
{

struct Path
{
  xt::xtensor<float, 1> x;
  xt::xtensor<float, 1> y;
  xt::xtensor<float, 1> yaws;

  void reset(unsigned int size)
  {
    x = xt::zeros<float>({size});
    y = xt::zeros<float>({size});
    yaws = xt::zeros<float>({size});
  }

}; 

}
// namespace mppi::models
