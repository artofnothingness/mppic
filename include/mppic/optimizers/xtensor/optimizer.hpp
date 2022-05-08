// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once

#include <string>
#include <memory>

// 3rdparty
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "mppic/optimizer_core_interface.hpp"

#include "mppic/optimizers/xtensor/motion_models.hpp"
#include "mppic/optimizers/xtensor/critic_manager.hpp"
#include "mppic/optimizers/xtensor/models/state.hpp"

namespace mppi::xtensor
{

class Optimizer : public IOptimizerCore
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

  span3d getGeneratedTrajectories() override;
  span2d getOptimizedTrajectory() override;

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
  xt::xtensor<double, 3> generateNoisedControls() const;

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
   *
   * @param trajectories costs, tensor of shape [ batch_size ]
   */
  void updateControlSequence(const xt::xtensor<double, 1> & costs);

  geometry_msgs::msg::TwistStamped
  getControlFromSequenceAsTwist(const builtin_interfaces::msg::Time & stamp);

  bool isHolonomic() const;

  void setOffset(double controller_frequency);

protected:
  models::State state_;
  models::ControlSequence control_sequence_;
  std::unique_ptr<MotionModel> motion_model_;
  CriticManager critic_manager_;

  xt::xtensor<double, 3> generated_trajectories_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi::xtensor
