#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "xtensor/xarray.hpp"
#include "xtensor/xview.hpp"

#include "mppic/Indexes.hpp"

namespace mppi::optimization {

template <typename T>
class Optimizer {
public:
  using model_t = xt::xtensor<T, 2>(const xt::xtensor<T, 2> &);

  Optimizer() = default;

  void on_configure(
      const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
      const std::string &node_name,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros,
      model_t &&model);

  void on_cleanup() {}
  void on_activate() {}
  void on_deactivate() {}

  geometry_msgs::msg::TwistStamped
  evalNextBestControl(const geometry_msgs::msg::PoseStamped &robot_pose,
                      const geometry_msgs::msg::Twist &robot_speed,
                      const nav_msgs::msg::Path &plan);

  xt::xtensor<T, 3> getGeneratedTrajectories() const {
    return generated_trajectories_;
  }

  xt::xtensor<T, 2> evalTrajectoryFromControlSequence(
      const geometry_msgs::msg::PoseStamped &robot_pose,
      const geometry_msgs::msg::Twist &robot_speed) const;

  double lineCost(int x0, int x1, int y0, int y1) const;

  double
  scoreFootprint(const std::vector<geometry_msgs::msg::Point> &footprint) const;

private:
  void getParams();
  void reset();

  /**
   * @brief Invoke generateNoisedControlBatches, assign result tensor to
   * batches_ controls dimensions and integrate recieved controls in
   * trajectories
   *
   * @return trajectories: tensor of shape [ batch_size_, time_steps_, 3 ] where
   * 3 stands for x, y, yaw
   */
  xt::xtensor<T, 3>
  generateNoisedTrajectories(const geometry_msgs::msg::PoseStamped &robot_pose,
                             const geometry_msgs::msg::Twist &robot_speed);

  /**
   * @brief Generate random controls by gaussian noise with mean in
   * control_sequence_
   *
   * @return Control batches tensor of shape [ batch_size_, time_steps_, 2]
   * where 2 stands for v, w
   */
  xt::xtensor<T, 3> generateNoisedControlBatches() const;

  void applyControlConstraints();

  /**
   * @brief Invoke setBatchesInitialVelocities and
   * propagateBatchesVelocitiesFromInitials
   *
   * @param twist current robot speed
   */
  void evalBatchesVelocities(const geometry_msgs::msg::Twist &robot_speed,
                             auto &batches) const;

  void setBatchesInitialVelocities(const geometry_msgs::msg::Twist &robot_speed,
                                   auto &batches) const;

  /**
   * @brief predict and propagate velocities in batches_ using model
   * for time horizont equal to time_steps_
   */
  void propagateBatchesVelocitiesFromInitials(auto &batches) const;

  xt::xtensor<T, 3> integrateBatchesVelocities(
      const geometry_msgs::msg::PoseStamped &robot_pose) const;

  xt::xtensor<T, 2>
  integrateSequence(const auto &velocities_sequence,
                    const geometry_msgs::msg::PoseStamped &robot_pose) const;

  /**
   * @brief Evaluate cost for each batch
   *
   * @param batches_of_trajectories batch of trajectories: tensor of shape [
   * batch_size_, time_steps_, 3 ] where 3 stands for x, y, yaw
   * @return Cost for each batch, tensor of shape [ batch_size ]
   */
  xt::xtensor<T, 1>
  evalBatchesCosts(const xt::xtensor<T, 3> &batches_of_trajectories,
                   const nav_msgs::msg::Path &global_plan,
                   const geometry_msgs::msg::PoseStamped &robot_pose) const;

  /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param batches_of_trajectories
   * @param costs [out] add reference cost values to this tensor
   */
  void evalReferenceCost(const auto &batches_of_trajectories,
                         const auto &global_plan, auto &costs) const;

  /**
   * @brief Evaluate cost related to trajectories path alignment using
   * approximate path to segment function
   *
   * @param batches_of_trajectories
   * @param costs [out] add reference cost values to this tensor
   */
  void evalApproxReferenceCost(const auto &batches_of_trajectories,
                               const auto &global_plan, auto &costs) const;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void evalGoalCost(const auto &batch_of_trajectories, const auto &global_plan,
                    auto &costs) const;

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @tparam B tensor type of batches trajectories
   * @tparam C costs type
   * @param costs [out] add obstacle cost values to this tensor
   */
  void evalObstacleCost(const auto &batch_of_trajectories, auto &costs) const;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose (considered
   * only if robot near last goal in current plan)
   *
   * @tparam P type of global plan (tensor like)
   * @tparam B tensor type of batches trajectories
   * @tparam C costs type
   * @param costs [out] add goal angle cost values to this tensor
   */
  void evalGoalAngleCost(const auto &batch_of_trajectories,
                         const auto &global_plan,
                         const geometry_msgs::msg::PoseStamped &robot_pose,
                         auto &costs) const;

  unsigned char costAtPose(const double x, const double y) const;
  bool inCollision(unsigned char cost) const;

  /**
   * @brief Update control_sequence_ with weighted by costs batch controls using
   * softmax function
   *
   * @param costs batches costs, tensor of shape [ batch_size ]
   */
  void updateControlSequence(const xt::xtensor<T, 1> &costs);

  std::vector<geometry_msgs::msg::Point> getOrientedFootprint(
      const std::array<double, 3> &robot_pose,
      const std::vector<geometry_msgs::msg::Point> &footprint_spec) const;

  /**
   * @brief Get offseted control from control_sequence_
   *
   */
  auto getControlFromSequence(unsigned int);

  auto getBatchesControls() const;
  auto getBatchesControls();

  auto getBatchesControlLinearVelocities() const;
  auto getBatchesControlLinearVelocities();

  auto getBatchesControlAngularVelocities() const;
  auto getBatchesControlAngularVelocities();

  auto getBatchesLinearVelocities() const;
  auto getBatchesLinearVelocities();

  auto getBatchesAngularVelocities() const;
  auto getBatchesAngularVelocities();

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::string node_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D *costmap_;
  std::function<model_t> model_;

  double inflation_cost_scaling_factor_;
  double inscribed_radius_;
  double inflation_radius_;

  double threshold_to_consider_goal_angle_;
  bool approx_reference_cost_;

  unsigned int batch_size_;
  unsigned int time_steps_;
  unsigned int iteration_count_;

  double model_dt_;
  double v_limit_;
  double w_limit_;
  double temperature_;
  T v_std_;
  T w_std_;

  unsigned int reference_cost_power_;
  unsigned int obstacle_cost_power_;
  unsigned int goal_cost_power_;
  unsigned int goal_angle_cost_power_;
  double reference_cost_weight_;
  double obstacle_cost_weight_;
  double goal_cost_weight_;
  double goal_angle_cost_weight_;

  static constexpr unsigned int batches_last_dim_size_ = 5;
  static constexpr unsigned int control_dim_size_ = 2;

  /**
   * @batches_ tensor of shape [ batch_size, time_steps, 5 ] where 5 stands for
   * robot linear, robot angluar velocities, linear control, angular control, dt
   * (time on which this control will be applied)
   */
  xt::xtensor<T, dims::batches> batches_;
  xt::xtensor<T, dims::batches> generated_trajectories_;

  /**
   * @control_sequence_ current best control sequence: tensor of shape [
   * time_steps, 2 ] where 2 stands for linear control, angular control
   */
  xt::xtensor<T, dims::control_sequence> control_sequence_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPI Optimizer")};
};

} // namespace mppi::optimization
