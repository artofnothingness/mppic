// Copyright 2022 FastSense, Samsung Research
#ifndef MPPIC__OPTIMIZER_HPP_
#define MPPIC__OPTIMIZER_HPP_

#include <string>
#include <memory>
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"

#include "mppic/motion_models.hpp"
#include "mppic/critic_manager.hpp"
#include "mppic/models/state.hpp"
#include "mppic/optimizer_core.hpp"

namespace mppi
{

class Optimizer : public OptimizerCore
{
public:
  using model_t = xt::xtensor<double, 2>(
    const xt::xtensor<double, 2> & state, const models::StateIdxes & idx);

  Optimizer() = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * dynamic_parameters_handler) override;


  geometry_msgs::msg::TwistStamped evalControl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed, const nav_msgs::msg::Path & plan,
    nav2_core::GoalChecker * goal_checker) override;

  xt::xtensor<double, 3> & getGeneratedTrajectories() override;

  xt::xtensor<double, 2> evalTrajectoryFromControlSequence(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed) override;


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
  xt::xtensor<double, 3> generateNoisedTrajectories(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed);

  /**
   * @brief Generate random controls by gaussian noise with mean in
   * control_sequence_
   *
   * @return tensor of shape [ batch_size_, time_steps_, 2]
   * where 2 stands for v, w
   */
  xt::xtensor<double, 3> generateNoisedControls() const;

  void applyControlConstraints();

  /**
   * @brief  Update velocities in state_
   *
   * @param twist current robot speed
   * @param state[out] fill state with velocities on each step
   */
  void updateStateVelocities(
    models::State & state,
    const geometry_msgs::msg::Twist & robot_speed) const;

  void
  updateInitialStateVelocities(
    models::State & state,
    const geometry_msgs::msg::Twist & robot_speed) const;

  /**
   * @brief predict velocities in state_ using model
   * for time horizont equal to time_steps_
   */
  void propagateStateVelocitiesFromInitials(models::State & state) const;

  xt::xtensor<double, 3> integrateStateVelocities(
    const models::State & state, const geometry_msgs::msg::PoseStamped & robot_pose) const;

  /**
   * @brief Update control_sequence_ with state controls weighted by costs
   * using softmax function
   *
   * @param trajectories costs, tensor of shape [ batch_size ]
   */
  void updateControlSequence(const xt::xtensor<double, 1> & costs);

  /**
   * @brief Get offseted control from control_sequence_
   *
   */
  auto getControlFromSequence(const unsigned int offset);
  geometry_msgs::msg::TwistStamped
  getControlFromSequenceAsTwist(
    const unsigned int offset,
    const builtin_interfaces::msg::Time & stamp);

  bool isHolonomic() const;

  void setOffset();

protected:
  models::State state_;
  models::ControlSequence control_sequence_;
  std::unique_ptr<MotionModel> motion_model_;
  CriticManager critic_manager_;

  xt::xtensor<double, 3> generated_trajectories_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // MPPIC__OPTIMIZER_HPP_
