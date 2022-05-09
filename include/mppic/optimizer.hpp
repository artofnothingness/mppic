// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#ifndef MPPIC__OPTIMIZER_HPP_
#define MPPIC__OPTIMIZER_HPP_

#include <string>
#include <memory>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "mppic/optimizers/xtensor/motion_models.hpp"
#include "mppic/optimizers/xtensor/critic_manager.hpp"
#include "mppic/optimizers/xtensor/models/state.hpp"

namespace mppi
{

class Optimizer
{
public:
  using model_t = xt::xtensor<double, 2>(
    const xt::xtensor<double, 2> & state, const models::StateIdxes & idx);

  Optimizer() = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * dynamic_parameters_handler);


  geometry_msgs::msg::TwistStamped evalControl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed, const nav_msgs::msg::Path & plan,
    nav2_core::GoalChecker * goal_checker);

  xtensor<double, 3> getGeneratedTrajectories();
  xtensor<double, 2> getOptimizedTrajectory();

  void setSpeedLimit(double speed_limit, bool percentage);

protected:
  void getParams();
  void reset();

  void setMotionModel(const std::string & model);

  void shiftControlSequence();

  /**
   *
   * @return trajectories: tensor of shape [ batch_size_, time_steps_, 3 ]
   * where 3 stands for x, y, yaw
   */
  xt::xtensor<double, 3> generateNoisedTrajectories();

  /**
   * @brief Generate random controls by gaussian noise with mean in
   * control_sequence_
   *
   * @return tensor of shape [ batch_size_, time_steps_, 2]
   * where 2 stands for v, w
   */
  void generateNoisedControls();

  void applyControlConstraints();

  /**
   * @brief  Update velocities in state_
   *
   * @param twist current robot speed
   * @param state[out] fill state with velocities on each step
   */
  void updateStateVelocities(models::State & state) const;

  void updateInitialStateVelocities(models::State & state) const;

  /**
   * @brief predict velocities in state_ using model
   * for time horizont equal to time_steps_
   */
  void propagateStateVelocitiesFromInitials(models::State & state) const;

  xt::xtensor<double, 3> integrateStateVelocities(const models::State & state) const;

  /**
   * @brief Update control_sequence_ with state controls weighted by costs
   * using softmax function
   */
  void updateControlSequence();

  geometry_msgs::msg::TwistStamped
  getControlFromSequenceAsTwist(const builtin_interfaces::msg::Time & stamp);

  bool isHolonomic() const;

  void setOffset(double controller_frequency);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string name_;

  ParametersHandler * parameters_handler_;

  std::unique_ptr<MotionModel> motion_model_;
  CriticManager critic_manager_;

  models::OptimizerSettings settings_;

  models::State state_;
  models::ControlSequence control_sequence_;

  xt::xtensor<double, 3> generated_trajectories_;
  xt::xtensor<double, 1> costs_;

  xt::xtensor<double, 3> vx_noises_;
  xt::xtensor<double, 3> vy_noises_;
  xt::xtensor<double, 3> wz_noises_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // MPPIC__OPTIMIZER_HPP_
