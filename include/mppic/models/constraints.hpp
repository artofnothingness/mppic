// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#ifndef MPPIC__MODELS__CONSTRAINTS_HPP_
#define MPPIC__MODELS__CONSTRAINTS_HPP_

namespace mppi::models
{

struct ControlConstraints
{
  double vx_max;
  double vx_min;
  double vy;
  double wz;
};

struct SamplingStd
{
  double vx;
  double vy;
  double wz;
};

}  // namespace mppi::models

#endif  // MPPIC__MODELS__CONSTRAINTS_HPP_
