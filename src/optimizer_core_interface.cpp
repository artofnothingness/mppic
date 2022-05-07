// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#include "mppic/optimizer_core_interface.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace mppi
{


void IOptimizerCore::setConstraints(double speed_limit, bool percentage)
{
  auto & s = settings_;
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    s.constraints.vx = s.base_constraints.vx;
    s.constraints.vy = s.base_constraints.vy;
    s.constraints.wz = s.base_constraints.wz;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      double ratio = speed_limit / 100.0;
      s.constraints.vx = s.base_constraints.vx * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    } else {
      // Speed limit is expressed in absolute value
      double ratio = speed_limit / s.base_constraints.vx;
      s.constraints.vx = speed_limit;
      s.constraints.vy = s.base_constraints.vx * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
  }
}

}  // namespace mppi
