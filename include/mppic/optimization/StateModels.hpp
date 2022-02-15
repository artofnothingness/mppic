#pragma once

#include <xtensor/xview.hpp>

namespace mppi::models {
/**
 * @brief Predict velocities for given trajectories the next time step
 *
 * @param state for given time_step, tensor of shape
 * [batch_size, ...] where last dim could be 5 or 7 depending on motion model used
 *
 * @return predicted velocities of the robot: tensor of shape [batch_size, ... ]
 * where last dim could be 2 or 3 depending on motion model used
 */
template<typename T, typename Tensor = xt::xtensor<T, 2>>
Tensor NaiveModel(const Tensor &state)
{
  return xt::view(state, xt::all(), xt::range(2, 4));
}

}// namespace mppi::models
