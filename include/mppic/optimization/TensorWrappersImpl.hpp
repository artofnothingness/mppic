#pragma once

#include "mppic/optimization/TensorWrappers.hpp"

namespace mppi::optimization {
template <typename T>
auto
State<T>::getVelocitiesVX() const
{
  return xt::view(data, xt::all(), xt::all(), idx.vx());
}

template <typename T>
auto
State<T>::getVelocitiesVX()
{
  return xt::view(data, xt::all(), xt::all(), idx.vx());
}

template <typename T>
auto
State<T>::getVelocitiesVY()
{
  return xt::view(data, xt::all(), xt::all(), idx.vy());
}

template <typename T>
auto
State<T>::getVelocitiesVY() const
{
  return xt::view(data, xt::all(), xt::all(), idx.vy());
}

template <typename T>
auto
State<T>::getVelocitiesWZ() const
{
  return xt::view(data, xt::all(), xt::all(), idx.wz());
}

template <typename T>
auto
State<T>::getVelocitiesWZ()
{
  return xt::view(data, xt::all(), xt::all(), idx.wz());
}

template <typename T>
auto
State<T>::getControlVelocitiesVX() const
{
  return xt::view(data, xt::all(), xt::all(), idx.cvx());
}

template <typename T>
auto
State<T>::getControlVelocitiesVX()
{
  return xt::view(data, xt::all(), xt::all(), idx.cvx());
}

template <typename T>
auto
State<T>::getControlVelocitiesVY()
{
  return xt::view(data, xt::all(), xt::all(), idx.cvy());
}

template <typename T>
auto
State<T>::getControlVelocitiesVY() const
{
  return xt::view(data, xt::all(), xt::all(), idx.cvy());
}

template <typename T>
auto
State<T>::getControlVelocitiesWZ() const
{
  return xt::view(data, xt::all(), xt::all(), idx.cwz());
}

template <typename T>
auto
State<T>::getControlVelocitiesWZ()
{
  return xt::view(data, xt::all(), xt::all(), idx.cwz());
}

template <typename T>
auto
State<T>::getTimeIntervals()
{
  return xt::view(data, xt::all(), xt::all(), idx.dt());
}

template <typename T>
auto
State<T>::getTimeIntervals() const
{
  return xt::view(data, xt::all(), xt::all(), idx.dt());
}

template <typename T>
auto
State<T>::getControls() const
{
  return xt::view(data, xt::all(), xt::all(), xt::range(idx.cbegin(), idx.cend()));
}

template <typename T>
auto
State<T>::getControls()
{
  return xt::view(data, xt::all(), xt::all(), xt::range(idx.cbegin(), idx.cend()));
}

template <typename T>
auto
State<T>::getVelocities() const
{
  return xt::view(data, xt::all(), xt::all(), xt::range(idx.vbegin(), idx.vend()));
}

template <typename T>
auto
State<T>::getVelocities()
{
  return xt::view(data, xt::all(), xt::all(), xt::range(idx.vbegin(), idx.vend()));
}

}  // namespace mppi::optimization
