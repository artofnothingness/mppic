#pragma once

#include "mppic/ControlSequence.hpp"

namespace mppi::optimization {

template <typename T>
void ControlSequence<T>::reset(std::vector<size_t> shape) {
  data = xt::zeros<T>(shape);
}

template <typename T>
auto
ControlSequence<T>::getControlLinearVelocities() {
  return xt::view(data, xt::all(), idx::cv);
}

template <typename T>
auto
ControlSequence<T>::getControlLinearVelocities() const {
  return xt::view(data, xt::all(), idx::cv);
}

template <typename T>
auto
ControlSequence<T>::getControlAngularVelocities() const {
  return xt::view(data, xt::all(), idx::cw);
}

template <typename T>
auto
ControlSequence<T>::getControlAngularVelocities() {
  return xt::view(data, xt::all(), idx::cw);
}

}  // namespace mppi::optimization
