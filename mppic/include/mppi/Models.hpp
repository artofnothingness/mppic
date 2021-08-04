#pragma once

#include <xtensor/xview.hpp>

namespace ultra::mppi::models {

template < 
  typename T, 
  typename Tensor = xt::xarray<T> 
>
Tensor NaiveModel(Tensor const& tensor) {
  return xt::view(tensor, xt::all(), xt::range(2, 4));
}

}
