#pragma once

#include <xtensor/xarray.hpp>
#include <xtensor/xstrided_view.hpp>
#include <xtensor/xview.hpp>

namespace mppi::optimization {

template <typename T>
struct State {
  struct idx {
    constexpr static uint8_t v = 0;
    constexpr static uint8_t w = 1;
    constexpr static uint8_t cv = 2;
    constexpr static uint8_t cw = 3;
    constexpr static uint8_t dt = 4;
    constexpr static std::array<uint8_t, 2> velocities_range{v, w + 1};
    constexpr static std::array<uint8_t, 2> controls_range{cv, cw + 1};
  };

  auto getControls() const;
  auto getControls();
  auto getVelocities();
  auto getVelocities() const;
  auto getControlLinearVelocities() const;
  auto getControlLinearVelocities();
  auto getControlAngularVelocities() const;
  auto getControlAngularVelocities();
  auto getLinearVelocities() const;
  auto getLinearVelocities();
  auto getAngularVelocities() const;
  auto getAngularVelocities();
  auto getTimeIntervals();
  auto getTimeIntervals() const;

  xt::xtensor<T, 3> data;
};

}  // namespace mppi::optimization
