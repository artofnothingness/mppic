// Copyright 2022 FastSense, Samsung Research

#pragma once

#include "mppic/models/constraints.hpp"

namespace mppi::models
{

struct OptimizerSettings
{
  models::ControlConstraints base_constraints_{0, 0, 0};
  models::ControlConstraints constraints_{0, 0, 0};
  models::SamplingStd sampling_std_{0, 0, 0};
  double model_dt_{0};
  double temperature_{0};
  unsigned int batch_size_{0};
  unsigned int time_steps_{0};
  unsigned int iteration_count_{0};
  int control_sequence_shift_offset_{0};
};

}  // namespace mppi::models
