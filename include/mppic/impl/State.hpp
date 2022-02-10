#pragma once

#include "mppic/State.hpp"

namespace mppi::optimization {

template <typename T, size_t dim>
auto State<T, dim>::getLinearVelocities() const {

  return xt::view(data, xt::all(), xt::all(), idx::v);
}

template <typename T, size_t dim> auto State<T, dim>::getLinearVelocities() {
  return xt::view(data, xt::all(), xt::all(), idx::v);
}

template <typename T, size_t dim>
auto State<T, dim>::getAngularVelocities() const {
  return xt::view(data, xt::all(), xt::all(), idx::w);
}

template <typename T, size_t dim> auto State<T, dim>::getAngularVelocities() {
  return xt::view(data, xt::all(), xt::all(), idx::w);
}

template <typename T, size_t dim>
auto State<T, dim>::getControlLinearVelocities() const {
  return xt::view(data, xt::all(), xt::all(), idx::cv);
}

template <typename T, size_t dim>
auto State<T, dim>::getControlLinearVelocities() {
  return xt::view(data, xt::all(), xt::all(), idx::cv);
}

template <typename T, size_t dim>
auto State<T, dim>::getControlAngularVelocities() const {
  return xt::view(data, xt::all(), xt::all(), idx::cw);
}

template <typename T, size_t dim>
auto State<T, dim>::getControlAngularVelocities() {
  return xt::view(data, xt::all(), xt::all(), idx::cw);
}

template <typename T, size_t dim> auto State<T, dim>::getTimeIntervals() {
  return xt::view(data, xt::all(), xt::all(), idx::dt);
}

template <typename T, size_t dim> auto State<T, dim>::getTimeIntervals() const {
  return xt::view(data, xt::all(), xt::all(), idx::dt);
}

template <typename T, size_t dim> auto State<T, dim>::getControls() const {
  return xt::view(data, xt::all(), xt::all(),
                  xt::range(idx::controls_range[0], idx::controls_range[1]));
}

template <typename T, size_t dim> auto State<T, dim>::getControls() {
  return xt::view(data, xt::all(), xt::all(),
                  xt::range(idx::controls_range[0], idx::controls_range[1]));
}

template <typename T, size_t dim> auto State<T, dim>::getVelocities() const {
  return xt::view(data, xt::all(), xt::all(),
                  xt::range(idx::velocities_range[0], idx::velocities_range[1]));
}

template <typename T, size_t dim> auto State<T, dim>::getVelocities() {
  return xt::view(data, xt::all(), xt::all(),
                  xt::range(idx::velocities_range[0], idx::velocities_range[1]));
}

} // namespace mppi::optimization
