#pragma once

#include <xtensor/xview.hpp>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"

namespace ultra::mppi::costs {
  using Costmap2DROS = nav2_costmap_2d::Costmap2DROS;
  using Path = nav_msgs::msg::Path;

  /**
   * @brief Evaluate next control
   *
   * @tparam T type of underlying values of tensor
   * @tparam Tensor tensor of shape [ batch_size, time_steps, 3 ] where 3 for x, y, yaw
   * @param tensor input batch
   * @return array of shape [ batch_size ] 
   **/
  template <typename T, typename Tensor = xt::xarray<T>>
  Tensor Cost(Tensor const& batch_of_trajectories, Path const& path, Costmap2DROS const& costmap) {
    (void)costmap;
    (void)batch_of_trajectories;
    (void)path;

    return {};
  }

}
