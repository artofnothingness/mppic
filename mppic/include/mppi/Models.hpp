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

  /* template < */ 
  /*   typename T, */
  /*   typename Tensor = xt::xarray<T> */ 
  /* > */ 
  /* static inline const std::map<std::string, std::function<Tensor(Tensor const&)>> Map = { */
  /*   {"Naive", &NaiveModel<T>}, */
  /* }; */

}
