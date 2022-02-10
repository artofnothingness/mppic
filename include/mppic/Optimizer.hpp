#pragma once

#include <tf2/utils.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xview.hpp>

#include "mppic/impl/State.hpp"

namespace mppi::optimization {

template <typename T>
class Optimizer {
public:
  static constexpr int state_dims = 2;
  using model_t = xt::xtensor<T, state_dims>(const xt::xtensor<T, state_dims> &);

  Optimizer() = default;

  void on_configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
                    const std::string &node_name,
                    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros,
                    model_t &&model);

  void
  on_cleanup() {}
  void
  on_activate() {}
  void
  on_deactivate() {}

  geometry_msgs::msg::TwistStamped evalNextBestControl(
      const geometry_msgs::msg::PoseStamped &robot_pose,
      const geometry_msgs::msg::Twist &robot_speed, const nav_msgs::msg::Path &plan);

  xt::xtensor<T, 3>
  getGeneratedTrajectories() const {
    return generated_trajectories_;
  }

  xt::xtensor<T, 2> evalTrajectoryFromControlSequence(
      const geometry_msgs::msg::PoseStamped &robot_pose,
      const geometry_msgs::msg::Twist &robot_speed) const;

  double lineCost(int x0, int x1, int y0, int y1) const;

  double scoreFootprint(const std::vector<geometry_msgs::msg::Point> &footprint) const;

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
  xt::xtensor<T, 3> generateNoisedTrajectories(const geometry_msgs::msg::PoseStamped &robot_pose,
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
  void evalBatchesVelocities(auto &state, const geometry_msgs::msg::Twist &robot_speed) const;

  void setBatchesInitialVelocities(auto &state, const geometry_msgs::msg::Twist &robot_speed) const;

  /**
   * @brief predict and propagate velocities in batches_ using model
   * for time horizont equal to time_steps_
   */
  void propagateBatchesVelocitiesFromInitials(auto &state) const;

  xt::xtensor<T, 3> integrateBatchesVelocities(
      const auto &state, const geometry_msgs::msg::PoseStamped &robot_pose) const;

  /**
   * @brief Evaluate cost for each batch
   *
   * @param batches_of_trajectories batch of trajectories: tensor of shape [
   * batch_size_, time_steps_, 3 ] where 3 stands for x, y, yaw
   * @return Cost for each batch, tensor of shape [ batch_size ]
   */
  xt::xtensor<T, 1> evalBatchesCosts(const xt::xtensor<T, 3> &batches_of_trajectories,
                                     const nav_msgs::msg::Path &global_plan,
                                     const geometry_msgs::msg::PoseStamped &robot_pose) const;

  /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param batches_of_trajectories
   * @param costs [out] add reference cost values to this tensor
   */
  void evalReferenceCost(const auto &batches_of_trajectories, const auto &global_plan,
                         auto &costs) const;

  /**
   * @brief Evaluate cost related to trajectories path alignment using
   * approximate path to segment function
   *
   * @param batches_of_trajectories
   * @param costs [out] add reference cost values to this tensor
   */
  void evalApproxReferenceCost(const auto &batches_of_trajectories, const auto &global_plan,
                               auto &costs) const;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void evalGoalCost(const auto &batch_of_trajectories, const auto &global_plan, auto &costs) const;

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  void evalObstacleCost(const auto &batch_of_trajectories, auto &costs) const;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose (considered
   * only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void evalGoalAngleCost(const auto &batch_of_trajectories, const auto &global_plan,
                         const geometry_msgs::msg::PoseStamped &robot_pose, auto &costs) const;

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

  State<T> state_;
  xt::xtensor<T, 3> generated_trajectories_;

  /**
   * @control_sequence_ current best control sequence: tensor of shape [
   * time_steps, 2 ] where 2 stands for linear control, angular control
   */
  xt::xtensor<T, 2> control_sequence_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPI Optimizer")};
};

}  // namespace mppi::optimization
