// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#ifndef MPPIC__MODELS__CONTROL_SEQUENCE_HPP_
#define MPPIC__MODELS__CONTROL_SEQUENCE_HPP_

#include <xtensor/xtensor.hpp>

namespace mppi::models
{

struct ControlSequence
{
  xt::xtensor<float, 1> vx;
  xt::xtensor<float, 1> vy;
  xt::xtensor<float, 1> wz;

  void reset(unsigned int time_steps) {
      vx = xt::zeros<float>({time_steps});
      vy = xt::zeros<float>({time_steps});
      wz = xt::zeros<float>({time_steps});
    }
};

}  // namespace mppi::models

#endif  // MPPIC__MODELS__CONTROL_SEQUENCE_HPP_
