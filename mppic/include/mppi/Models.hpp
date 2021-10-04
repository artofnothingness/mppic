#pragma once

#include <xtensor/xview.hpp>

namespace mppi::models {

/**
 * @brief Predict velocities for the next time step from current time step batches of robot state
 *
 * @tparam T underlying tensor type
 * @param batches batches of robot state for concrete time_step, tensor of shape [batch_size, 5] where 5 stands for
  * robot linear, angluar velocities, linear control, angular control velocities, dt (time on which this control will be applied)
 * @return predicted velocities of the robot: tensor of shape [batch_size, 2] where 2 stands for
  * robot linear, angluar velocities for the next time step

 */
template<typename T, typename Tensor = xt::xtensor<T, 2>>
auto NaiveModel(const Tensor &batches)
  -> Tensor
{
  return xt::view(batches, xt::all(), xt::range(2, 4));
}

}// namespace mppi::models
