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
          typename Model = Tensor(Tensor const &),
          typename Cost = Tensor(Tensor const &, Path const &,
                                 std::shared_ptr<Costmap2DROS> const &)>
class Optimizer {
public:
  Optimizer() = default;
  ~Optimizer() = default;

  Optimizer(LifecycleNode::SharedPtr const &parent, string const &node_name,
            shared_ptr<Buffer> const &tf,
            shared_ptr<Costmap2DROS> const &costmap_ros, Model &&model) {

    parent_ = parent;
    costmap_ros_ = costmap_ros;
    tf_buffer_ = tf;
    node_name_ = node_name;

    model_ = model;

    using namespace utils;
    getParam("model_dt", 0.1, parent_, model_dt_);
    getParam("time_steps", 20, parent_, time_steps_);
    getParam("batch_size", 100, parent_, batch_size_);
    getParam("std_v", 0.1, parent_, std_v_);
    getParam("std_w", 0.1, parent_, std_w_);
    getParam("limit_v", 0.5, parent_, limit_v_);
    getParam("limit_w", 1.0, parent_, limit_w_);
    getParam("iteration_count", 2, parent_, iteration_count_);
    getParam("lookahead_dist", 1.2, parent_, lookagead_dist_);
    getParam("temperature", 0.25, parent_, temperature_);
    resetBatches();
  }

  void setPlan(Path const &path) { global_plan_ = path; }

  void resetBatches() {
    batches_ = xt::zeros<float>({batch_size_, time_steps_, last_dim_});

    xt::view(batches_, xt::all(), xt::all(), 4) = model_dt_;

    control_sequence_ = xt::zeros<float>({time_steps_, control_dim_size_});
  }

  /**
   * @brief Evaluate next control
   *
   * @param Pose current pose of the robot
   * @param Twist Current speed of the rosbot
   * @return Next control
   */
  auto evalNextControl(PoseStamped const &pose, Twist const &twist)
      -> TwistStamped {

    auto transformed_plan = transformGlobalPlan(pose);

    for (int i = 0; i < iteration_count_; ++i) {
      Tensor trajectories = generateNoisedTrajectoryBatches(pose, twist);
      Tensor costs = evalBatchesCosts(trajectories);
      updateControlSequence(costs);
    }

    return getControlFromSequence(pose.header);
  };

private:
  Tensor generateNoisedTrajectoryBatches(PoseStamped const &pose,
                                         Twist const &twist) {
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

  void setBatchesVelocity(Twist const &twist) {
    setBatchesInitialVelocities(twist);
    propagateBatchesVelocityFromInitials();
  }

  void setBatchesInitialVelocities(Twist const &twist) {
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

  auto integrateVelocityBatches(PoseStamped const &robot_pose) const -> Tensor {

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

  Tensor evalBatchesCosts(Tensor const &trajectory_batches) const {
    std::vector<size_t> shape = {trajectory_batches.shape()[0]};

    auto obstacle_cost = xt::zeros<T>(shape);
    auto reference_cost = xt::zeros<T>(shape);
    auto goal_cost = xt::zeros<T>(shape);

    return obstacle_cost + reference_cost + goal_cost;
  }

  template <typename Iter, typename Stamp>
  Path getTransformedToLocalPlan(Iter begin, Iter end, Stamp const &stamp) {

    auto transform_pose = [&](auto const &global_plan_pose) {
      PoseStamped global_pose;
      PoseStamped local_pose;

      global_pose.header.frame_id = global_plan_.header.frame_id;
      global_pose.header.stamp = stamp;
      global_pose.pose = global_plan_pose.pose;

      transformPose(costmap_ros_->getBaseFrameID(), global_pose, local_pose);
      return local_pose;
    };

    Path plan;
    std::transform(begin, end, std::back_inserter(plan.poses), transform_pose);

    plan.header.frame_id = costmap_ros_->getBaseFrameID();
    plan.header.stamp = stamp;

    return plan;
  }

  auto pruneGlobalPlan(auto const &end) {
    global_plan_.poses.erase(global_plan_.poses.begin(), end);
  }

  Path transformGlobalPlan(PoseStamped const &pose) {

    auto robot_pose = transformToGlobalFrame(pose);
    auto const &stamp = robot_pose.header.stamp;
    double max_dist = getMaxTransformDistance();

    auto &&[constrained_begin, constrained_end] =
        getLocalPlanBounds(robot_pose, max_dist);

    auto transformed_plan =
        getTransformedToLocalPlan(constrained_begin, constrained_end, stamp);

    pruneGlobalPlan(constrained_begin);

    /* global_path_pub_->publish(transformed_plan); */

    if (transformed_plan.poses.empty())
      throw std::runtime_error("Resulting plan has 0 poses in it.");

    return transformed_plan;
  }

  bool transformPose(string const &frame, PoseStamped const &in_pose,
                     PoseStamped &out_pose) const {

    if (in_pose.header.frame_id == frame) {
      out_pose = in_pose;
      return true;
    }

    try {
      tf_buffer_->transform(in_pose, out_pose, frame,
                            tf2::durationFromSec(transform_tolerance_));
      out_pose.header.frame_id = frame;
      return true;
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(parent_->get_logger(), "Exception in transformPose: %s",
                   ex.what());
    }
    return false;
  }

  auto transformToGlobalFrame(PoseStamped const &pose) -> PoseStamped {

    if (global_plan_.poses.empty()) {
      throw std::runtime_error("Received plan with zero length");
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
      throw std::runtime_error(
          "Unable to transform robot pose into global plan's frame");
    }

    return robot_pose;
  }

  auto getLocalPlanBounds(PoseStamped const &pose, double max_transform_dist) {

    auto begin = global_plan_.poses.begin();
    auto end = global_plan_.poses.end();

    auto closest_point = std::min_element(
        begin, end, [&pose](PoseStamped const& lhs, PoseStamped const& rhs) {
          return utils::hypot(lhs, pose) < utils::hypot(rhs, pose);
        });

    // Find points definitely outside of the costmap so we won't transform them.
    auto outside_costmap_point = std::find_if(
        closest_point, end, [&](PoseStamped const &global_plan_pose) {
          return utils::hypot(pose, global_plan_pose) > max_transform_dist;
        });

    return std::tuple{closest_point, outside_costmap_point};
  }

  double getMaxTransformDistance() {
    Costmap2D *costmap = costmap_ros_->getCostmap();

    return std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
           costmap->getResolution() / 2.0;
  }

  void updateControlSequence(Tensor costs) {
    costs = costs - xt::amin(costs);
    auto exponents = xt::exp(-1 / temperature_ * costs);
    auto softmaxes = exponents / xt::sum(exponents);
    auto softmaxes_expanded =
        xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

    control_sequence_ = xt::sum(getControlBatches() * softmaxes_expanded, 0);
  }

  TwistStamped getControlFromSequence(auto const &header) {
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
  LifecycleNode::SharedPtr parent_;
  Path global_plan_;
  std::string node_name_;
  std::shared_ptr<Costmap2DROS> costmap_ros_;
  std::shared_ptr<Buffer> tf_buffer_;

  static int constexpr last_dim_ = 5;
  static int constexpr control_dim_size_ = 2;
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

  static double constexpr transform_tolerance_ = 0.1;
};

} // namespace ultra::mppi::optimization
