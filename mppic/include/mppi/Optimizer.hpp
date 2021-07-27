#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "xtensor/xadapt.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor/xslice.hpp"
#include <xtensor/xmath.hpp>

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
  -> xt::xarray<double> {

    auto v_noises =
        xt::random::randn<double>({batch_size, time_steps}, 0.0, v_std);
    auto w_noises =
        xt::random::randn<double>({batch_size, time_steps}, 0.0, w_std);

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

    auto linear_v_batches = xt::view(batches, xt::all(), xt::all(), 0); // batch_size x time_steps
    auto angular_v_batches = xt::view(batches, xt::all(), xt::all(), 1); // batch_size x time_steps

    auto angular_offsets = angular_v_batches * model_dt; // batch_size x time_steps
    auto rotation_batches = xt::cumsum(angular_offsets, 1); // batch_size x time_steps

    rotation_batches -= xt::view(rotation_batches, xt::all(), xt::range(_, 1)); //  batch_size x time_steps
    rotation_batches += robot_yaw;

    auto v_x = linear_v_batches * xt::cos(rotation_batches);
    auto v_y = linear_v_batches * xt::sin(rotation_batches);

    auto x = xt::cumsum(v_x * model_dt, 1);
    auto y = xt::cumsum(v_y * model_dt, 1);
    x = robot_x - xt::view(x, xt::all(), xt::range(_, 1));
    y = robot_y - xt::view(x, xt::all(), xt::range(_, 1));

    x = xt::view(x, xt::all(), xt::all(), xt::newaxis());
    y = xt::view(y, xt::all(), xt::all(), xt::newaxis());
    auto yaw = xt::view(rotation_batches, xt::all(), xt::all(), xt::newaxis());

    return xt::concatenate(xtuple(x, y, yaw), 2);
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
  std::function<xt::xarray<float>(xt::xarray<float>)> model;
};

} // namespace ultra::mppi
