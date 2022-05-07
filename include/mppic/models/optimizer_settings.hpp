// Copyright 2022 FastSense, Samsung Research

#pragma once

#include "mppic/models/constraints.hpp"

namespace mppi::models
{

struct OptimizerSettings
{
  models::ControlConstraints base_constraints{0, 0, 0};
  models::ControlConstraints constraints{0, 0, 0};
  models::SamplingStd sampling_std{0, 0, 0};
  double model_dt{0};
  double temperature{0};
  unsigned int batch_size{0};
  unsigned int time_steps{0};
  unsigned int iteration_count{0};
  bool shift_control_sequence = false;
};

}  // namespace mppi::models
