#pragma once

#include "mppic/State.hpp"

namespace mppi::optimization {

template <typename T>
auto
State<T>::getLinearVelocities() const {
  return xt::view(data, xt::all(), xt::all(), idx::v);
}

template <typename T>
auto
State<T>::getLinearVelocities() {
  return xt::view(data, xt::all(), xt::all(), idx::v);
}

template <typename T>
auto
State<T>::getAngularVelocities() const {
  return xt::view(data, xt::all(), xt::all(), idx::w);
}

template <typename T>
auto
State<T>::getAngularVelocities() {
  return xt::view(data, xt::all(), xt::all(), idx::w);
}

template <typename T>
auto
State<T>::getControlLinearVelocities() const {
  return xt::view(data, xt::all(), xt::all(), idx::cv);
}

template <typename T>
auto
State<T>::getControlLinearVelocities() {
  return xt::view(data, xt::all(), xt::all(), idx::cv);
}

template <typename T>
auto
State<T>::getControlAngularVelocities() const {
  return xt::view(data, xt::all(), xt::all(), idx::cw);
}

template <typename T>
auto
State<T>::getControlAngularVelocities() {
  return xt::view(data, xt::all(), xt::all(), idx::cw);
}

template <typename T>
auto
State<T>::getTimeIntervals() {
  return xt::view(data, xt::all(), xt::all(), idx::dt);
}

template <typename T>
auto
State<T>::getTimeIntervals() const {
  return xt::view(data, xt::all(), xt::all(), idx::dt);
}

template <typename T>
auto
State<T>::getControls() const {
  return xt::view(data, xt::all(), xt::all(),
                  xt::range(idx::controls_range[0], idx::controls_range[1]));
}

template <typename T>
auto
State<T>::getControls() {
  return xt::view(data, xt::all(), xt::all(),
                  xt::range(idx::controls_range[0], idx::controls_range[1]));
}

template <typename T>
auto
State<T>::getVelocities() const {
  return xt::view(data, xt::all(), xt::all(),
                  xt::range(idx::velocities_range[0], idx::velocities_range[1]));
}

template <typename T>
auto
State<T>::getVelocities() {
  return xt::view(data, xt::all(), xt::all(),
                  xt::range(idx::velocities_range[0], idx::velocities_range[1]));
}

}  // namespace mppi::optimization
