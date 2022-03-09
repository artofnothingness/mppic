#pragma once

#include <array>
#include <cstdint>

#include <xtensor/xtensor.hpp>

#include "mppic/optimization/MotionModel.hpp"

namespace mppi::optimization {

class ControlSequnceIdxes
{
public:
  unsigned int dim() const { return dim_; }

  uint8_t vx() const { return vx_; }
  uint8_t vy() const { return vy_; }
  uint8_t wz() const { return wz_; }

  void setLayout(MotionModel motion_model)
  {
    if (isHolonomic(motion_model)) {
      vx_ = 0;
      vy_ = 1;
      wz_ = 2;
      dim_ = 3;
    } else {
      vx_ = 0;
      wz_ = 1;
      dim_ = 2;
    }
  }

private:
  uint8_t vx_{0};
  uint8_t vy_{0};
  uint8_t wz_{0};
  unsigned int dim_{0};
};

struct ControlSequence
{
  xt::xtensor<double, 2> data;
  ControlSequnceIdxes idx;

  void reset(unsigned int time_steps) { data = xt::zeros<double>({time_steps, idx.dim()}); }
};

} // namespace mppi::optimization
