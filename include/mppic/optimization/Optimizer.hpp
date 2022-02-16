#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2/utils.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <xtensor/xarray.hpp>
#include <xtensor/xview.hpp>

#include "mppic/optimization/CriticScorerImpl.hpp"
#include "mppic/optimization/MotionModel.hpp"
#include "mppic/optimization/TensorWrappersImpl.hpp"

namespace mppi::optimization {
template <typename T>
class Optimizer
{
public:
  using model_t = xt::xtensor<T, 2>(const xt::xtensor<T, 2> &);

  Optimizer() = default;

  void on_configure(
    rclcpp_lifecycle::LifecycleNode * parent, const std::string & node_name,
    nav2_costmap_2d::Costmap2DROS * costmap_ros, model_t model);

  void
  on_cleanup()
  {}
  void
  on_activate()
  {}
  void
  on_deactivate()
  {}

  geometry_msgs::msg::TwistStamped evalControl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed, const nav_msgs::msg::Path & plan);

  xt::xtensor<T, 3>
  getGeneratedTrajectories() const
  {
    return generated_trajectories_;
  }

  xt::xtensor<T, 2> evalTrajectoryFromControlSequence(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed) const;

private:
  void getParams();
  void reset();
  void configureComponents();

  MotionModel getMotionModel() const;
  void setMotionModel(MotionModel);

  /**
   *
   * @return trajectories: tensor of shape [ batch_size_, time_steps_, 3 ]
   * where 3 stands for x, y, yaw
   */
  xt::xtensor<T, 3> generateNoisedTrajectories(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed);

  /**
   * @brief Generate random controls by gaussian noise with mean in
   * control_sequence_
   *
   * @return tensor of shape [ batch_size_, time_steps_, 2]
   * where 2 stands for v, w
   */
  xt::xtensor<T, 3> generateNoisedControls() const;

  void applyControlConstraints();

  /**
   * @brief  Update velocities in state_
   *
   * @param twist current robot speed
   * @param state[out] fill state with velocities on each step
   */
  void updateStateVelocities(auto & state, const geometry_msgs::msg::Twist & robot_speed) const;

  void
  updateInitialStateVelocities(auto & state, const geometry_msgs::msg::Twist & robot_speed) const;

  /**
   * @brief predict velocities in state_ using model
   * for time horizont equal to time_steps_
   */
  void propagateStateVelocitiesFromInitials(auto & state) const;

  xt::xtensor<T, 3> integrateStateVelocities(
    const auto & state, const geometry_msgs::msg::PoseStamped & robot_pose) const;

  /**
   * @brief Update control_sequence_ with state controls weighted by costs
   * using softmax function
   *
   * @param trajectories costs, tensor of shape [ batch_size ]
   */
  void updateControlSequence(const xt::xtensor<T, 1> & costs);

  std::vector<geometry_msgs::msg::Point> getOrientedFootprint(
    const std::array<double, 3> & robot_pose,
    const std::vector<geometry_msgs::msg::Point> & footprint_spec) const;

  /**
   * @brief Get offseted control from control_sequence_
   *
   */
  auto getControlFromSequence(unsigned int);

  rclcpp_lifecycle::LifecycleNode * parent_{nullptr};
  std::string node_name_;
  nav2_costmap_2d::Costmap2DROS * costmap_ros_{nullptr};
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  unsigned int batch_size_{0};
  unsigned int time_steps_{0};
  unsigned int iteration_count_{0};
  double model_dt_{0};
  double temperature_{0};
  double vx_max_{0};
  double vy_max_{0};
  double wz_max_{0};
  T vx_std_{0};
  T vy_std_{0};
  T wz_std_{0};
  bool approx_reference_cost_{true};

  State<T> state_;
  ControlSequence<T> control_sequence_;
  CriticScorer<T> critic_scorer_;
  MotionModel motion_model_t_{MotionModel::DiffDrive};
  std::function<model_t> model_;

  xt::xtensor<T, 3> generated_trajectories_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPI Optimizer")};
};

} // namespace mppi::optimization
