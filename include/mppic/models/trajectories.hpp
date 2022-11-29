// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#ifndef MPPIC__MODELS__TRAJECTORIES_HPP_
#define MPPIC__MODELS__TRAJECTORIES_HPP_

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

namespace mppi::models
{

/**
 * @class mppi::models::Trajectories
 * @brief Candidate Trajectories
 */
struct Trajectories
{
  xt::xtensor<float, 2> x;
  xt::xtensor<float, 2> y;
  xt::xtensor<float, 2> yaws;

  /**
    * @brief Reset state data
    */
  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    x = xt::zeros<float>({batch_size, time_steps});
    y = xt::zeros<float>({batch_size, time_steps});
    yaws = xt::zeros<float>({batch_size, time_steps});
  }

  /**
    * @brief Get the last point in all trajectories
    * @return A tensor containing only final points in trajectories
    */
  inline auto getLastPoints() const
  {
    return xt::concatenate(
      xtuple(
        xt::view(x, xt::all(), -1, xt::newaxis()),
        xt::view(y, xt::all(), -1, xt::newaxis())), 1);
  }
};

}  // namespace mppi::models

#endif  // MPPIC__MODELS__TRAJECTORIES_HPP_
