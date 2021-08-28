#pragma once

#include <xtensor/xview.hpp>

namespace mppi::models {

template <typename T, typename Tensor = xt::xarray<T>> auto 
NaiveModel(const Tensor &tensor) 
-> Tensor
{
  return xt::view(tensor, xt::all(), xt::range(2, 4));
}

} // namespace mppi::models
