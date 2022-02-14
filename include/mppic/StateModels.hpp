#pragma once

#include <xtensor/xview.hpp>

namespace mppi::models {
/**
 * @brief Predict velocities for given trajectories the next time step
 *
 * @param state for given time_step, tensor of shape
 * [batch_size, 5] where 5 stands for robot linear, angluar velocities, linear
 * control, angular control velocities, dt (time on which this control will be
 * applied)
 * @return predicted velocities of the robot: tensor of shape [batch_size, 2]
 * where 2 stands for robot linear, angluar velocities for the next time step
 */
template<typename T, typename Tensor = xt::xtensor<T, 2>>
Tensor NaiveModel(const Tensor &state)
{
  return xt::view(state, xt::all(), xt::range(2, 4));
}

}// namespace mppi::models
