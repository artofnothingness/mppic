#pragma once

#include "mppic/State.hpp"

namespace mppi::optimization {

template <typename T>
auto
State<T>::getVelocitiesVX() const {
  return xt::view(data, xt::all(), xt::all(), idx.vx);
}

template <typename T>
auto
State<T>::getVelocitiesVX() {
  return xt::view(data, xt::all(), xt::all(), idx.vx);
}

template <typename T>
auto
State<T>::getVelicitiesWZ() const {
  return xt::view(data, xt::all(), xt::all(), idx.wz);
}

template <typename T>
auto
State<T>::getVelicitiesWZ() {
  return xt::view(data, xt::all(), xt::all(), idx.wz);
}

template <typename T>
auto
State<T>::getControlVelocitiesVX() const {
  return xt::view(data, xt::all(), xt::all(), idx.cvx);
}

template <typename T>
auto
State<T>::getControlVelocitiesVX() {
  return xt::view(data, xt::all(), xt::all(), idx.cvx);
}

template <typename T>
auto
State<T>::getControlVelocitiesWZ() const {
  return xt::view(data, xt::all(), xt::all(), idx.cwz);
}

template <typename T>
auto
State<T>::getControlVelocitiesWZ() {
  return xt::view(data, xt::all(), xt::all(), idx.cwz);
}

template <typename T>
auto
State<T>::getTimeIntervals() {
  return xt::view(data, xt::all(), xt::all(), idx.dt);
}

template <typename T>
auto
State<T>::getTimeIntervals() const {
  return xt::view(data, xt::all(), xt::all(), idx.dt);
}

template <typename T>
auto
State<T>::getControls() const {
  return xt::view(data, xt::all(), xt::all(),
                  xt::range(idx.controls_range[0], idx.controls_range[1]));
}

template <typename T>
auto
State<T>::getControls() {
  return xt::view(data, xt::all(), xt::all(),
                  xt::range(idx.controls_range[0], idx.controls_range[1]));
}

template <typename T>
auto
State<T>::getVelocities() const {
  return xt::view(data, xt::all(), xt::all(),
                  xt::range(idx.velocities_range[0], idx.velocities_range[1]));
}

template <typename T>
auto
State<T>::getVelocities() {
  return xt::view(data, xt::all(), xt::all(),
                  xt::range(idx.velocities_range[0], idx.velocities_range[1]));
}

}  // namespace mppi::optimization
