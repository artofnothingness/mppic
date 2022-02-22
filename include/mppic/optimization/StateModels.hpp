#pragma once

#include <xtensor/xview.hpp>

#include "mppic/optimization/tensor_wrappers/StateImpl.hpp"

namespace mppi::optimization::models {
/**
 * @brief Predict velocities for given trajectories the next time step
 *
 * @param state for given time_step, tensor of shape
 * [batch_size, ...] where last dim could be 5 or 7 depending on motion model used
 *
 * @return predicted velocities of the robot: tensor of shape [batch_size, ... ]
 * where last dim could be 2 or 3 depending on motion model used
 */
template <typename T>
xt::xtensor<T, 2> NaiveModel(const xt::xtensor<T, 2> & state, const StateIdxes & idx)
{
  return xt::view(state, xt::all(), xt::range(idx.cbegin(), idx.cend()));
}

} // namespace mppi::optimization::models
