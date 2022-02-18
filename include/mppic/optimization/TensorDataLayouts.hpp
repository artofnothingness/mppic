#pragma once

#include <cstdint>

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

class StateIdxes
{
public:
  uint8_t vbegin() const { return vrange_[0]; }
  uint8_t vend() const { return vrange_[1]; }
  uint8_t vx() const { return vx_; }
  uint8_t vy() const { return vy_; }
  uint8_t wz() const { return wz_; }

  uint8_t cbegin() const { return crange_[0]; }
  uint8_t cend() const { return crange_[1]; }
  uint8_t cvx() const { return cvx_; }
  uint8_t cvy() const { return cvy_; }
  uint8_t cwz() const { return cwz_; }

  uint8_t dt() const { return dt_; };
  unsigned int dim() const { return dim_; }

  void setLayout(MotionModel motion_model)
  {
    if (isHolonomic(motion_model)) {
      vx_ = 0;
      vy_ = 1;
      wz_ = 2;
      cvx_ = 3;
      cvy_ = 4;
      cwz_ = 5;
      dt_ = 6;
      dim_ = 7;
    } else {
      vx_ = 0;
      wz_ = 1;
      cvx_ = 2;
      cwz_ = 3;
      dt_ = 4;
      dim_ = 5;
    }

    vrange_[0] = vx_;
    vrange_[1] = cvx_;
    crange_[0] = cvx_;
    crange_[1] = dt_;
  }

private:
  uint8_t vx_{0};
  uint8_t vy_{0};
  uint8_t wz_{0};
  uint8_t cvx_{0};
  uint8_t cvy_{0};
  uint8_t cwz_{0};
  uint8_t dt_{0};
  std::array<uint8_t, 2> vrange_{0, 0};
  std::array<uint8_t, 2> crange_{0, 0};

  unsigned int dim_{0};
};

} // namespace mppi::optimization
