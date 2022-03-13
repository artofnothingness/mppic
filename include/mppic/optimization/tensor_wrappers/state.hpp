#ifndef MPPIC__OPTIMIZATION__TENSOR_WRAPPERS__STATE_HPP_
#define MPPIC__OPTIMIZATION__TENSOR_WRAPPERS__STATE_HPP_

#include <array>
#include <cstdint>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xarray.hpp>

#include "mppic/optimization/motion_model.hpp"

namespace mppi::optimization
{

class StateIdxes
{
public:
  uint8_t vbegin() const {return vrange_[0];}
  uint8_t vend() const {return vrange_[1];}
  uint8_t vx() const {return vx_;}
  uint8_t vy() const {return vy_;}
  uint8_t wz() const {return wz_;}

  uint8_t cbegin() const {return crange_[0];}
  uint8_t cend() const {return crange_[1];}
  uint8_t cvx() const {return cvx_;}
  uint8_t cvy() const {return cvy_;}
  uint8_t cwz() const {return cwz_;}

  uint8_t dt() const {return dt_;}
  unsigned int dim() const {return dim_;}

  void setLayout(MotionModel motion_model)
  {
    // Layout changes to include "Y" components if holonomic
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

struct State
{
  xt::xtensor<double, 3> data;
  StateIdxes idx;

  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    data = xt::zeros<double>({batch_size, time_steps, idx.dim()});
  }

  auto getVelocitiesVX() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.vx());
  }

  auto getVelocitiesVX()
  {
    return xt::view(data, xt::all(), xt::all(), idx.vx());
  }

  auto getVelocitiesVY()
  {
    return xt::view(data, xt::all(), xt::all(), idx.vy());
  }

  auto getVelocitiesVY() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.vy());
  }

  auto getVelocitiesWZ() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.wz());
  }

  auto getVelocitiesWZ()
  {
    return xt::view(data, xt::all(), xt::all(), idx.wz());
  }

  auto getControlVelocitiesVX() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.cvx());
  }

  auto getControlVelocitiesVX()
  {
    return xt::view(data, xt::all(), xt::all(), idx.cvx());
  }

  auto getControlVelocitiesVY()
  {
    return xt::view(data, xt::all(), xt::all(), idx.cvy());
  }

  auto getControlVelocitiesVY() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.cvy());
  }

  auto getControlVelocitiesWZ() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.cwz());
  }

  auto getControlVelocitiesWZ()
  {
    return xt::view(data, xt::all(), xt::all(), idx.cwz());
  }

  auto getTimeIntervals()
  {
    return xt::view(data, xt::all(), xt::all(), idx.dt());
  }

  auto getTimeIntervals() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.dt());
  }

  auto getControls() const
  {
    return xt::view(data, xt::all(), xt::all(), xt::range(idx.cbegin(), idx.cend()));
  }

  auto getControls()
  {
    return xt::view(data, xt::all(), xt::all(), xt::range(idx.cbegin(), idx.cend()));
  }

  auto getVelocities() const
  {
    return xt::view(data, xt::all(), xt::all(), xt::range(idx.vbegin(), idx.vend()));
  }

  auto getVelocities()
  {
    return xt::view(data, xt::all(), xt::all(), xt::range(idx.vbegin(), idx.vend()));
  }
};

} // namespace mppi::optimization

#endif  // MPPIC__OPTIMIZATION__TENSOR_WRAPPERS__STATE_HPP_
