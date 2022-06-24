// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include <array>
#include <cstdint>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xrandom.hpp>

#include "mppic/models/optimizer_settings.hpp"

namespace mppi
{

/**
 * @brief Keeps named indexes of noises last dimension variables
 */
class NoisesIdxes
{
public:
  uint8_t vbegin() const {return velocity_range_[0];}
  uint8_t vend() const {return velocity_range_[1];}
  uint8_t vx() const {return vx_;}
  uint8_t vy() const {return vy_;}
  uint8_t wz() const {return wz_;}

  unsigned int dim() const {return dim_;}

  bool isHolonomic()
  {
    return is_holonomic_;
  }

  void setLayout(const bool is_holonomic)
  {
    is_holonomic_ = is_holonomic;
    // Layout changes to include "Y" components if holonomic
    if (is_holonomic_) {
      vx_ = 0;
      vy_ = 1;
      wz_ = 2;
      dim_ = 3;
    } else {
      vx_ = 0;
      wz_ = 1;
      dim_ = 2;
    }

    velocity_range_[0] = vx_;
    velocity_range_[1] = dim_;
  }

private:
  uint8_t vx_{0};
  uint8_t vy_{0};
  uint8_t wz_{0};
  std::array<uint8_t, 2> velocity_range_{0, 0};
  bool is_holonomic_{false};

  unsigned int dim_{0};
};

struct NoiseGenerator
{
  auto getNoiseVX() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.vx());
  }

  auto getNoiseVX()
  {
    return xt::view(data, xt::all(), xt::all(), idx.vx());
  }

  auto getNoiseVY()
  {
    return xt::view(data, xt::all(), xt::all(), idx.vy());
  }

  auto getNoiseVY() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.vy());
  }

  auto getNoiseWZ() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.wz());
  }

  auto getNoiseWZ()
  {
    return xt::view(data, xt::all(), xt::all(), idx.wz());
  }

  auto getVelocities()
  {
    return xt::view(data, xt::all(), xt::all(), xt::range(idx.vbegin(), idx.vend()));
  }

  xt::xtensor<double, 3> const & getNoises() const { return data;};

  xt::xtensor<double, 3> const & generate() {
    auto & s = optimizer_settings_;
    getNoiseVX() = xt::random::randn<double>({s.batch_size, s.time_steps, 1U}, 0.0, s.sampling_std.vx);

    getNoiseWZ() = xt::random::randn<double>({s.batch_size, s.time_steps, 1U}, 0.0, s.sampling_std.wz);

    if (idx.isHolonomic()) {
      getNoiseVY() = xt::random::randn<double>({s.batch_size, s.time_steps, 1U}, 0.0, s.sampling_std.vy);
    } 

    return getNoises();
  }


  void reset(const models::OptimizerSettings & optimizer_settings, bool is_holonomic)
  {
    idx.setLayout(is_holonomic);
    optimizer_settings_ = optimizer_settings;
    data = xt::zeros<double>({optimizer_settings_.batch_size, optimizer_settings_.time_steps, idx.dim()});
  }

private:
models::OptimizerSettings optimizer_settings_;
xt::xtensor<double, 3> data;
NoisesIdxes idx;
};


}  // namespace mppi::models
