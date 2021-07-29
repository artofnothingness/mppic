#pragma once
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

#include "xtensor/xadapt.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor/xslice.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "mppi/Utils.hpp"

namespace ultra::mppi::optimization {

template <
  typename T, 
  typename Tensor = xt::xarray<T> 
>
class Optimizer {
public:
  using ManagedNode = rclcpp_lifecycle::LifecycleNode;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Twist = geometry_msgs::msg::Twist;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Path = nav_msgs::msg::Path;

  Optimizer() = default;
  ~Optimizer() = default;

  void configure(ManagedNode::SharedPtr const& parent) {
    m_parent = parent;
    using namespace utils;

    getParam("model_dt", 0.1, m_parent, m_model_dt);
    getParam("time_steps", 20, m_parent, m_time_steps);
    getParam("batch_size", 100, m_parent, m_batch_size);
    getParam("std_v", 0.1, m_parent, m_std_v);
    getParam("std_w", 0.1, m_parent, m_std_w);
    getParam("limit_v", 0.5, m_parent, m_limit_v);
    getParam("limit_w", 1.0, m_parent, m_limit_w);
    getParam("iteration_count", 2, m_parent, m_iterations_count);
    getParam("lookahead_dist", 1.2, m_parent, m_lookahead_dist);

    reset();
  };

  /**
   * @brief Calculate next control
   *
   * @param Pose current pose of the robot
   * @param Twist Current speed of the rosbot
   * @param Path Current global path
   * @return Next control
   */
  auto calculateNextControl(PoseStamped const& pose, Twist const& twist, Path const& path)
  -> TwistStamped {

    for (int i = 0; i < m_iterations_count; ++i) {
      Tensor trajectories = getNextTrajectories(pose, twist, path);
      Tensor costs = evalBatchesCosts(trajectories, path);
      updateControlSequence(costs); // TODO
    }

    return TwistStamped{};
  };

  void reset() {
    m_batches = xt::zeros<float>({m_batch_size, m_time_steps, m_last_dim});
    xt::view(m_batches, xt::all(), xt::all(), 4) = m_model_dt;

    m_control_sequence = xt::zeros<float>({m_time_steps, m_control_dim});
  }

private:

  void updateControlSequence(Tensor) {

  }

  Tensor evalBatchesCosts(Tensor const& batches, Path const& path) {
    return m_cost(batches, path); //TODO create functors 
  }

  Tensor getNextTrajectories(PoseStamped const& pose, Twist const& twist) {
    setInitialVelocities(twist);
    getControlSequences() = generateNoisedControlSequences();
    applyControlConstraints();
    propagateVelocities();
    return integrateVelocities(pose, twist);
  }

  void setInitialVelocities(Twist const& twist) {
    xt::view(m_batches, xt::all(), 0, 0) = twist.linear.x;
    xt::view(m_batches, xt::all(), 0, 1) = twist.angular.z;
  }

  void applyControlConstraints(){
    xt::clip(getLinearVelocityControlSequence(), -m_limit_v, m_limit_v);
    xt::clip(getAngularVelocityControlSequence(), -m_limit_w, m_limit_w);
  }

  void propagateVelocities() {
    for (int t = 0; t < m_time_steps - 1; t++) {
      auto curr_batch = xt::view(m_batches, xt::all(), t);
      auto next_batch = xt::view(m_batches, xt::all(), t + 1);
      next_batch = m_model(curr_batch);
    }
  }

  decltype(auto) getControlSequences() {
    return xt::view(m_batches, xt::all(), xt::all(), xt::range(2, 4));
  }

  decltype(auto) getLinearVelocityControlSequence() {
    return xt::view(m_batches, xt::all(), xt::all(), 2);
  }

  decltype(auto) getAngularVelocityControlSequence() {
    return xt::view(m_batches, xt::all(), xt::all(), 3);
  }

  auto generateNoisedControlSequences() 
  -> Tensor {

    auto v_noises = xt::random::randn<T>({m_batch_size, m_time_steps}, 0.0, m_std_v);
    auto w_noises = xt::random::randn<T>({m_batch_size, m_time_steps}, 0.0, m_std_w);

    return m_control_sequence + xt::concatenate(xtuple(v_noises, w_noises), 2);
  }

  auto integrateVelocities(PoseStamped const& robot_pose, Twist const& robot_twist) 
  -> Tensor {

    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

    auto v = xt::view(m_batches, xt::all(), xt::all(), 0);
    auto w = xt::view(m_batches, xt::all(), xt::all(), 1);

    auto d_yaw = w * m_model_dt;
    auto yaw = xt::cumsum(d_yaw, 1);
    yaw -= xt::view(yaw, xt::all(), 0, xt::newaxis());
    yaw += robot_yaw;

    auto d_x = v * xt::cos(yaw) * m_model_dt;
    auto d_y = v * xt::sin(yaw) * m_model_dt;

    auto x = xt::cumsum(d_x, 1);
    auto y = xt::cumsum(d_y, 1);
    x = robot_x - xt::view(x, xt::all(), 0, xt::newaxis());
    y = robot_y - xt::view(x, xt::all(), 0, xt::newaxis());

    // Add axis
    x = xt::view(x, xt::all(), xt::all(), xt::newaxis());
    y = xt::view(y, xt::all(), xt::all(), xt::newaxis());
    yaw = xt::view(yaw, xt::all(), xt::all(), xt::newaxis());

    return xt::concatenate(xtuple(x, y, yaw), 2);
  }

public:
  ManagedNode::SharedPtr m_parent;

  static int constexpr m_last_dim = 5;
  static int constexpr m_control_dim = 2;
  int m_time_steps;
  int m_batch_size;

  double m_model_dt;
  double m_std_v;
  double m_std_w;
  double m_limit_w;
  double m_limit_v;

  int m_iterations_count;
  double m_lookahead_dist;

  Tensor m_batches;
  Tensor m_control_sequence;

  using model_t = Tensor(Tensor);
  std::function<model_t> m_model;

  using cost_t = Tensor(Tensor const&, Path const&);
  std::function<cost_t> m_cost;
};

} // namespace ultra::mppi
