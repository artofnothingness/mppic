// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#ifndef MPPIC__MODELS__PATH_HPP_
#define MPPIC__MODELS__PATH_HPP_

#include <xtensor/xtensor.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::Path
 * @brief Path represented as a tensor
 */
struct Path
{
  xt::xtensor<float, 1> x;
  xt::xtensor<float, 1> y;
  xt::xtensor<float, 1> yaws;

  /**
    * @brief Reset path data
    */
  void reset(unsigned int size)
  {
    x = xt::zeros<float>({size});
    y = xt::zeros<float>({size});
    yaws = xt::zeros<float>({size});
  }
};

}  // namespace mppi::models

#endif  // MPPIC__MODELS__PATH_HPP_
