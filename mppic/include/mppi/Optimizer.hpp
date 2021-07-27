#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "xtensor/xadapt.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor/xslice.hpp"

namespace ultra::mppi {

class Optimizer {
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Twist = geometry_msgs::msg::Twist;
  using Path = nav_msgs::msg::Path;

public:
  Optimizer() = default;
  ~Optimizer() = default;

  /**
   * @brief Calculate next control
   *
   * @param Pose current pose of the robot
   * @param Twist Current speed of the rosbot
   * @param Path Current global path
   * @return Next control
   */
  auto operator()(PoseStamped const &pose, Twist const &twist, Path const &path)
      -> TwistStamped;

  void reset() {
    batches = xt::zeros<float>({batch_size, time_steps, last_dim});
    xt::view(batches, xt::all(), xt::all(), 4) = model_dt;

    control_sequence = xt::zeros<float>({time_steps, control_dim});
  }

  decltype(auto) generateNoise() {
    auto v_noises = xt::random::randn<double>({batch_size, time_steps}, 0.0, v_std);
    auto w_noises = xt::random::randn<double>({batch_size, time_steps}, 0.0, w_std);

    return xt::concatenate(xtuple(v_noises, w_noises), 2);
  }

  auto updateBatches() {
    auto noises = generateNoise();

    xt::view(batches, xt::all(), xt::all(), xt::range(2, 4)) = control_sequence + noises;
    xt::clip(xt::view(batches, xt::all(), xt::all(), 2),
        -v_limit, v_limit);

    xt::clip(xt::view(batches, xt::all(), xt::all(), 3),
        -w_limit, w_limit);

  }



  static constexpr int last_dim = 5;
  static constexpr int control_dim = 2;
  int time_steps;
  int batch_size;

  double model_dt;
  double v_std;
  double w_std;

  double v_limit;
  double w_limit;

  int n_optimizations;
  double lookahead_dist;

  xt::xarray<float> batches;
  xt::xarray<float> control_sequence;
};

} // namespace ultra::mppi
