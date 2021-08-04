#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "xtensor/xarray.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor/xslice.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "mppi/Utils.hpp"

namespace ultra::mppi::optimization {

using std::shared_ptr;
using std::string;

using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::Costmap2DROS;
using rclcpp_lifecycle::LifecycleNode;

using tf2_ros::Buffer;

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

  Optimizer(const shared_ptr<LifecycleNode> &parent, const string &node_name,
            const shared_ptr<Buffer> &tf,
            const shared_ptr<Costmap2DROS> &costmap_ros, Model &&model) {

    parent_ = parent;
    costmap_ros_ = costmap_ros;
    tf_buffer_ = tf;
    model_ = model;

    using namespace utils;
    getParam(node_name + ".model_dt", 0.1, parent_, model_dt_);
    getParam(node_name + ".time_steps", 20, parent_, time_steps_);
    getParam(node_name + ".batch_size", 100, parent_, batch_size_);
    getParam(node_name + ".std_v", 0.1, parent_, std_v_);
    getParam(node_name + ".std_w", 0.1, parent_, std_w_);
    getParam(node_name + ".limit_v", 0.5, parent_, limit_v_);
    getParam(node_name + ".limit_w", 1.0, parent_, limit_w_);
    getParam(node_name + ".iteration_count", 2, parent_, iteration_count_);
    getParam(node_name + ".lookahead_dist", 1.2, parent_, lookagead_dist_);
    getParam(node_name + ".temperature", 0.25, parent_, temperature_);
    resetBatches();
  }

  void resetBatches() {
    batches_ = xt::zeros<float>({batch_size_, time_steps_, last_dim_});

    xt::view(batches_, xt::all(), xt::all(), 4) = model_dt_;

    control_sequence_ = xt::zeros<float>({time_steps_, control_dim_size_});
  }

  /**
   * @brief Evaluate next control
   *
   * @param Pose current pose of the robot
   * @param Twist current speed of the rosbot
   * @param Path global plan
   * @return Next control
   */
  /**
   * @brief Evaluate next control
   *
   * @param pose current pose of the robot
   * @param twist curent speed of the robot
   * @param path global plan
   * @return best control
   */
  auto evalNextControl(const PoseStamped &pose, const Twist &twist,
                       const Path &path) -> TwistStamped {

    for (int i = 0; i < iteration_count_; ++i) {
      Tensor trajectories = generateNoisedTrajectoryBatches(pose, twist);
      Tensor costs = evalBatchesCosts(trajectories, path);
      updateControlSequence(costs);
    }

    return getControlFromSequence(pose.header);
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

  Tensor evalBatchesCosts(const Tensor &trajectory_batches,
                          const Path &path) const {
    (void)path;

    std::vector<size_t> shape = {trajectory_batches.shape()[0]};

    auto obstacle_cost = xt::zeros<T>(shape);
    auto reference_cost = xt::zeros<T>(shape);
    auto goal_cost = xt::zeros<T>(shape);

    return obstacle_cost + reference_cost + goal_cost;
  }

  void updateControlSequence(Tensor &costs) {
    costs = costs - xt::amin(costs);
    auto exponents = xt::exp(-1 / temperature_ * costs);
    auto softmaxes = exponents / xt::sum(exponents);
    auto softmaxes_expanded =
        xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

    control_sequence_ = xt::sum(getControlBatches() * softmaxes_expanded, 0);
  }

  TwistStamped getControlFromSequence(const auto &header) {
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
  shared_ptr<LifecycleNode> parent_;
  shared_ptr<Costmap2DROS> costmap_ros_;
  shared_ptr<Buffer> tf_buffer_;

  static constexpr int last_dim_ = 5;
  static constexpr int control_dim_size_ = 2;
  int time_steps_;
  int batch_size_;

  double model_dt_;
  double std_v_;
  double std_w_;
  double limit_w_;
  double limit_v_;

  int iteration_count_;
  double lookagead_dist_;

  double temperature_;

  Tensor batches_;
  Tensor control_sequence_;

  std::function<Model> model_;

  static constexpr double transform_tolerance_ = 0.1;
};

} // namespace ultra::mppi::optimization
