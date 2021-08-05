#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "xtensor/xarray.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor/xslice.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "mppi/Utils.hpp"

namespace ultra::mppi::optimization {

using nav2_costmap_2d::Costmap2D;

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Path;

template <typename T, typename Tensor = xt::xarray<T>,
          typename Model = Tensor(const Tensor &)>
class Optimizer {
public:
  Optimizer() = default;
  ~Optimizer() = default;

  Optimizer(int batch_size, double std_v, double std_w, double limit_v,
            double limit_w, double model_dt, int time_steps,
            int iteration_count, double temperature, Model model)
      : batch_size_(batch_size), time_steps_(time_steps),
        iteration_count_(iteration_count), model_dt_(model_dt), std_v_(std_v),
        std_w_(std_w), limit_v_(limit_v), limit_w_(limit_w),
        temperature_(temperature), model_(model) {

    resetBatches();
  }

  void resetBatches() {
    batches_ = xt::zeros<float>({batch_size_, time_steps_, last_dim_});
    control_sequence_ = xt::zeros<float>({time_steps_, control_dim_size_});
    xt::view(batches_, xt::all(), xt::all(), 4) = model_dt_;
  }

  /**
   * @brief Evaluate next control
   *
   * @param pose current pose of the robot
   * @param twist curent speed of the robot
   * @param path global plan
   * @param costmap costmap
   * @return best control
   */
  auto evalNextControl(const PoseStamped &pose, const Twist &twist,
                       const Path &path, const Costmap2D &costmap)
      -> TwistStamped {

    for (int i = 0; i < iteration_count_; ++i) {
      Tensor trajectories = generateNoisedTrajectoryBatches(pose, twist);
      Tensor costs = evalBatchesCosts(trajectories, path, costmap);
      updateControlSequence(costs);
    }

    TwistStamped cmd = getControlFromSequence(pose.header);

    RCLCPP_INFO(logger_, "Send Speed { %f %: }", cmd.twist.linear.x, cmd.twist.angular.z);
    return cmd;
  };

private:
  Tensor generateNoisedTrajectoryBatches(const PoseStamped &pose,
                                         const Twist &twist) {
    getControlBatches() = generateNoisedControlBatches();
    applyControlConstraints();
    setBatchesVelocity(twist);
    return integrateVelocityBatches(pose);
  }

  auto generateNoisedControlBatches() -> Tensor {
    auto v_noises =
        xt::random::randn<T>({batch_size_, time_steps_, 1}, 0.0, std_v_);
    auto w_noises =
        xt::random::randn<T>({batch_size_, time_steps_, 1}, 0.0, std_w_);
    return control_sequence_ + xt::concatenate(xtuple(v_noises, w_noises), 2);
  }

  void applyControlConstraints() {
    auto v = getLinearVelocityControlBatches();
    auto w = getAngularVelocityControlBatches();
    v = xt::clip(v, -limit_v_, limit_v_);
    w = xt::clip(w, -limit_w_, limit_w_);
  }

  void setBatchesVelocity(const Twist &twist) {
    setBatchesInitialVelocities(twist);
    propagateBatchesVelocityFromInitials();
  }

  void setBatchesInitialVelocities(const Twist &twist) {
    xt::view(batches_, xt::all(), 0, 0) = twist.linear.x;
    xt::view(batches_, xt::all(), 0, 1) = twist.angular.z;
  }

  void propagateBatchesVelocityFromInitials() {
    using namespace xt::placeholders;

    for (int t = 0; t < time_steps_ - 1; t++) {
      auto curr_batch = xt::view(batches_, xt::all(), t); // -> batch x 5
      auto next_batch_velocities =
          xt::view(batches_, xt::all(), t + 1, xt::range(_, 2)); // batch x 2

      next_batch_velocities = model_(curr_batch);
    }
  }

  auto integrateVelocityBatches(const PoseStamped &robot_pose) const -> Tensor {

    using namespace xt::placeholders;

    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

    auto v = xt::view(batches_, xt::all(), xt::all(), 0);
    auto w = xt::view(batches_, xt::all(), xt::all(), 1);

    auto yaw = xt::cumsum(w * model_dt_, 1);
    yaw -= xt::view(yaw, xt::all(), xt::range(_, 1));

    yaw += robot_yaw;

    auto x = xt::cumsum(v * xt::cos(yaw) * model_dt_, 1);
    auto y = xt::cumsum(v * xt::sin(yaw) * model_dt_, 1);

    x += robot_x - xt::view(x, xt::all(), xt::range(_, 1));
    y += robot_y - xt::view(x, xt::all(), xt::range(_, 1));

    return xt::concatenate(
        xtuple(xt::view(x, xt::all(), xt::all(), xt::newaxis()),
               xt::view(y, xt::all(), xt::all(), xt::newaxis()),
               xt::view(yaw, xt::all(), xt::all(), xt::newaxis())),
        2);
  }

  Tensor evalBatchesCosts(const Tensor &trajectory_batches, const Path &path,
                          const Costmap2D /*&costmap*/) const {

    std::vector<size_t> shape = {trajectory_batches.shape()[0]};

    auto reference_cost = xt::zeros<T>(shape);
    auto obstacle_cost = xt::zeros<T>(shape);

    auto goal_cost = [&](int weight, int power) {
      Tensor last_goal = {static_cast<T>(path.poses.back().pose.position.x),
                          static_cast<T>(path.poses.back().pose.position.y)};

      Tensor x = xt::view(trajectory_batches, xt::all(), xt::all(), 0);
      Tensor y = xt::view(trajectory_batches, xt::all(), xt::all(), 1);

      auto dx = x - last_goal[0];
      auto dy = y - last_goal[1];

      auto dists = xt::hypot(dx, dy);

      auto result = weight * xt::pow(xt::view(dists, xt::all(), -1), power);
      return Tensor(result);
    };

    auto result = obstacle_cost + reference_cost + goal_cost(2, 2);

    return result;
  }

  void updateControlSequence(Tensor &costs) {
    costs = costs - xt::amin(costs);
    auto exponents = xt::exp(-1 / temperature_ * costs);
    auto softmaxes = exponents / xt::sum(exponents);
    auto softmaxes_expanded =
        xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

    control_sequence_ = xt::sum(getControlBatches() * softmaxes_expanded, 0);
  }

  template <typename H> TwistStamped getControlFromSequence(const H &header) {
    return utils::toTwistStamped(xt::view(control_sequence_, 0), header);
  }

  decltype(auto) getControlBatches() {
    return xt::view(batches_, xt::all(), xt::all(), xt::range(2, 4));
  }

  decltype(auto) getLinearVelocityControlBatches() {
    return xt::view(batches_, xt::all(), xt::all(), 2);
  }

  decltype(auto) getAngularVelocityControlBatches() {
    return xt::view(batches_, xt::all(), xt::all(), 3);
  }

private:
  static constexpr int last_dim_ = 5;
  static constexpr int control_dim_size_ = 2;

  int batch_size_;
  int time_steps_;
  int iteration_count_;

  double model_dt_;
  double std_v_;
  double std_w_;
  double limit_v_;
  double limit_w_;
  double temperature_;

  Tensor batches_;
  Tensor control_sequence_;
  std::function<Model> model_;

  rclcpp::Logger logger_{rclcpp::get_logger("Optimizer")};
};

} // namespace ultra::mppi::optimization
