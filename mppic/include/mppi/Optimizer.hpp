#pragma once

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

#include "xtensor/xadapt.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor/xslice.hpp"
#include <xtensor/xmath.hpp>

namespace ultra::mppi::optimization {

template <
  typename T, 
  typename Container = xt::xarray<T> 
>
class Optimizer {
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Twist = geometry_msgs::msg::Twist;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
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
  auto operator()(PoseStamped const& pose, Twist const& twist, Path const& path)
  -> TwistStamped;

  void reset() {
    batches = xt::zeros<float>({batch_size, time_steps, last_dim});
    xt::view(batches, xt::all(), xt::all(), 4) = model_dt;

    control_sequence = xt::zeros<float>({time_steps, control_dim});
  }

  void generateTrajectories () {
    updateBatches();
  }

  void updateBatches() {
    auto controls           = xt::view(batches, xt::all(), xt::all(), xt::range(2, 4));
    auto linear_velocities  = xt::view(batches, xt::all(), xt::all(), 2);
    auto angular_velocities = xt::view(batches, xt::all(), xt::all(), 3);

    controls = control_sequence + generateNoise();

    linear_velocities = xt::clip(linear_velocities, -v_limit, v_limit);
    angular_velocities = xt::clip(angular_velocities, -w_limit, w_limit);

    predictVelocities();
  }

  auto generateNoise() 
  -> Container {

    auto v_noises =
        xt::random::randn<T>({batch_size, time_steps}, 0.0, v_std);
    auto w_noises =
        xt::random::randn<T>({batch_size, time_steps}, 0.0, w_std);

    return xt::concatenate(xtuple(v_noises, w_noises), 2);
  }

  void predictVelocities() {
    for (int t = 0; t < time_steps - 1; t++) {
      auto curr_batch = xt::view(batches, xt::all(), t);
      auto next_batch = xt::view(batches, xt::all(), t + 1);
      next_batch = model(curr_batch);
    }
  }

  auto integrateVelocities(double robot_x, double robot_y, double robot_yaw) {
    using namespace xt::placeholders;

    auto v = xt::view(batches, xt::all(), xt::all(), 0);
    auto w = xt::view(batches, xt::all(), xt::all(), 1);

    auto d_yaw = w * model_dt;
    auto yaw = xt::cumsum(d_yaw, 1);
    yaw -= xt::view(yaw, xt::all(), xt::range(_, 1));
    yaw += robot_yaw;

    auto d_x = v * xt::cos(yaw) * model_dt;
    auto d_y = v * xt::sin(yaw) * model_dt;

    auto x = xt::cumsum(d_x, 1);
    auto y = xt::cumsum(d_y, 1);
    x = robot_x - xt::view(x, xt::all(), xt::range(_, 1));
    y = robot_y - xt::view(x, xt::all(), xt::range(_, 1));

    // Add axis
    x = xt::view(x, xt::all(), xt::all(), xt::newaxis());
    y = xt::view(y, xt::all(), xt::all(), xt::newaxis());
    yaw = xt::view(yaw, xt::all(), xt::all(), xt::newaxis());

    return xt::concatenate(xtuple(x, y, yaw), 2);
  }

  static int constexpr last_dim = 5;
  static int constexpr control_dim = 2;
  int time_steps;
  int batch_size;

  double model_dt;
  double v_std;
  double w_std;

  double v_limit;
  double w_limit;

  int n_optimizations;
  double lookahead_dist;

  Container batches;
  Container control_sequence;

  using functor_t = Container(Container);
  std::function<functor_t> model;
};

} // namespace ultra::mppi
