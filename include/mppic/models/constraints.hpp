// Copyright 2022 FastSense, Samsung Research
#ifndef MPPIC__MODELS__CONSTRAINTS_HPP_
#define MPPIC__MODELS__CONSTRAINTS_HPP_

namespace mppi::models
{

struct ControlConstraints
{
  double vx;
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
