// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include "mppic/models/constraints.hpp"
#include <cstddef>

namespace mppi::models
{

struct OptimizerSettings
{
  models::ControlConstraints base_constraints{0, 0, 0, 0};
  models::ControlConstraints constraints{0, 0, 0, 0};
  models::ControlConstraints d_constraints{0, 0, 0, 0};
  models::SamplingStd sampling_std{0, 0, 0};
  float model_dt{0};
  float action_dt{0};
  float temperature{0};
  unsigned int batch_size{0};
  unsigned int time_steps{0};
  unsigned int iteration_count{0};
  bool shift_control_sequence{false};
  size_t retry_attempt_limit{0};
};

}  // namespace mppi::models
