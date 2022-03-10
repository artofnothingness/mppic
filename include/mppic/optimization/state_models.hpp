#pragma once

#include <xtensor/xview.hpp>

#include "mppic/optimization/tensor_wrappers/state.hpp"

namespace mppi
{
/**
 * @brief Predict velocities for given trajectories the next time step
 *
 * @param state for given time_step, tensor of shape
 * [batch_size, ...] where last dim could be 5 or 7 depending on motion model used
 *
 * @return predicted velocities of the robot: tensor of shape [batch_size, ... ]
 * where last dim could be 2 or 3 depending on motion model used
 */
xt::xtensor<double, 2> NaiveModel(const xt::xtensor<double, 2> & state, const optimization::StateIdxes & idx)
{
  return xt::view(state, xt::all(), xt::range(idx.cbegin(), idx.cend()));
}

} // namespace mppi
