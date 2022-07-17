// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#ifndef MPPIC__MODELS__STATE_HPP_
#define MPPIC__MODELS__STATE_HPP_

#include <array>
#include <cstdint>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xarray.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace mppi::models
{

/**
 * @brief Keeps named indexes of state last dimension variables
 */
class StateIdxes
{
public:
  uint8_t vbegin() const {return velocity_range_[0];}
  uint8_t vend() const {return velocity_range_[1];}
  unsigned int vdim() const {return vdim_;}
  uint8_t vx() const {return vx_;}
  uint8_t vy() const {return vy_;}
  uint8_t wz() const {return wz_;}

  uint8_t cbegin() const {return control_range_[0];}
  uint8_t cend() const {return control_range_[1];}
  unsigned int cdim() const {return cdim_;}
  uint8_t cvx() const {return cvx_;}
  uint8_t cvy() const {return cvy_;}
  uint8_t cwz() const {return cwz_;}

  void setLayout(const bool is_holonomic)
  {
    // Layout changes to include "Y" components if holonomic
    if (is_holonomic) {
      vx_ = 0;
      vy_ = 1;
      wz_ = 2;
      velocity_range_[0] = 0;
      velocity_range_[1] = 3;
      vdim_ = 3u;

      cvx_ = 0;
      cvy_ = 1;
      cwz_ = 2;
      control_range_[0] = 0;
      control_range_[1] = 3;
      cdim_ = 3u;

    } else {
      vx_ = 0;
      wz_ = 1;
      velocity_range_[0] = 0;
      velocity_range_[1] = 2;
      vdim_ = 2u;

      cvx_ = 0;
      cwz_ = 1;
      control_range_[0] = 0;
      control_range_[1] = 2;
      cdim_ = 2u;
    }
  }

private:
  uint8_t vx_{0};
  uint8_t vy_{0};
  uint8_t wz_{0};
  unsigned int vdim_{0};

  uint8_t cvx_{0};
  uint8_t cvy_{0};
  uint8_t cwz_{0};
  unsigned int cdim_{0};

  std::array<uint8_t, 2> velocity_range_{0, 0};
  std::array<uint8_t, 2> control_range_{0, 0};
};

/**
 * @brief State represent current the state of optimization problem.
 *
 * State stores state of the system for each trajectory. It has shape [ batch_size x time_steps x dim ].
 * Last dimension described by StateIdxes and consists of velocities, controls,
 * and amount of time between time steps (vx, [vy], wz, cvx, [cvy], cwz, dt)
 *
 **/
struct State
{
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist speed;
  xt::xtensor<float, 3> velocities;
  xt::xtensor<float, 3> controls;
  xt::xtensor<float, 2> dt;
  StateIdxes idx;

  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    velocities = xt::zeros<float>({batch_size, time_steps, idx.vdim()});
    controls = xt::zeros<float>({batch_size, time_steps, idx.cdim()});
    dt = xt::zeros<float>({batch_size, time_steps});
  }

  auto getVelocitiesVX() const
  {
    return xt::view(velocities, xt::all(), xt::all(), idx.vx());
  }

  auto getVelocitiesVX()
  {
    return xt::view(velocities, xt::all(), xt::all(), idx.vx());
  }

  auto getVelocitiesVY()
  {
    return xt::view(velocities, xt::all(), xt::all(), idx.vy());
  }

  auto getVelocitiesVY() const
  {
    return xt::view(velocities, xt::all(), xt::all(), idx.vy());
  }

  auto getVelocitiesWZ() const
  {
    return xt::view(velocities, xt::all(), xt::all(), idx.wz());
  }

  auto getVelocitiesWZ()
  {
    return xt::view(velocities, xt::all(), xt::all(), idx.wz());
  }

  auto getVelocities() const
  {
    return xt::view(velocities, xt::all(), xt::all(), xt::all());
  }

  auto getVelocities()
  {
    return xt::view(velocities, xt::all(), xt::all(), xt::all());
  }

  auto getControlVelocitiesVX() const
  {
    return xt::view(controls, xt::all(), xt::all(), idx.cvx());
  }

  auto getControlVelocitiesVX()
  {
    return xt::view(controls, xt::all(), xt::all(), idx.cvx());
  }

  auto getControlVelocitiesVY()
  {
    return xt::view(controls, xt::all(), xt::all(), idx.cvy());
  }

  auto getControlVelocitiesVY() const
  {
    return xt::view(controls, xt::all(), xt::all(), idx.cvy());
  }

  auto getControlVelocitiesWZ() const
  {
    return xt::view(controls, xt::all(), xt::all(), idx.cwz());
  }

  auto getControlVelocitiesWZ()
  {
    return xt::view(controls, xt::all(), xt::all(), idx.cwz());
  }

  auto getControls() const
  {
    return xt::view(controls, xt::all(), xt::all(), xt::all());
  }

  auto getControls()
  {
    return xt::view(controls, xt::all(), xt::all(), xt::all());
  }

  auto getTimeIntervals() const
  {
    return xt::view(dt, xt::all(), xt::all());
  }

  auto getTimeIntervals()
  {
    return xt::view(dt, xt::all(), xt::all());
  }
};


}  // namespace mppi::models

#endif  // MPPIC__MODELS__STATE_HPP_
