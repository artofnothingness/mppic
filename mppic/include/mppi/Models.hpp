#pragma once

#include <xtensor/xview.hpp>

namespace mppi::models {

template <typename T, typename Tensor = xt::xarray<T>>
Tensor NaiveModel(const Tensor &tensor) {
  return xt::view(tensor, xt::all(), xt::range(2, 4));
}

} // namespace ultra::mppi::models
