#pragma once

#include <xtensor/xarray.hpp>
#include <xtensor/xstrided_view.hpp>
#include <xtensor/xview.hpp>

namespace mppi::optimization {

template <typename T>
struct ControlSequence {
  struct idx {
    constexpr static uint8_t cv = 0;
    constexpr static uint8_t cw = 1;
  };
  
  void reset(std::vector<size_t> shape);

  auto getControlLinearVelocities() const;
  auto getControlLinearVelocities();
  auto getControlAngularVelocities() const;
  auto getControlAngularVelocities();

  xt::xtensor<T, 2> data;
};

}  // namespace mppi::optimization
