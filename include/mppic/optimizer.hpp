// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#ifndef MPPIC__OPTIMIZER_HPP_
#define MPPIC__OPTIMIZER_HPP_

#include <string>
#include <memory>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/goal_checker.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "mppic/models/optimizer_settings.hpp"
#include "mppic/motion_models.hpp"
#include "mppic/critic_manager.hpp"
#include "mppic/models/state.hpp"
#include "mppic/models/trajectories.hpp"
#include "mppic/models/path.hpp"
#include "mppic/tools/noise_generator.hpp"
#include "mppic/tools/parameters_handler.hpp"
#include "mppic/tools/utils.hpp"

namespace mppi
{

class Optimizer
{
public:
  Optimizer() = default;

  ~Optimizer() {shutdown();}

  void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * dynamic_parameters_handler);

  void shutdown();

  geometry_msgs::msg::TwistStamped evalControl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed, const nav_msgs::msg::Path & plan,
    nav2_core::GoalChecker * goal_checker);

  models::Trajectories & getGeneratedTrajectories();
  xt::xtensor<float, 2> getOptimizedTrajectory();

  void setSpeedLimit(double speed_limit, bool percentage);

protected:
  void optimize();

  void prepare(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker);

  void getParams();
  void reset();

  void setMotionModel(const std::string & model);

  void shiftControlSequence();

  void shiftActionSequence();

  /**
   *
   * @brief updates generated_trajectories_
   */
  void generateNoisedTrajectories();

  void applyVelocityConstraints();

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

  void integrateStateVelocities(
    models::Trajectories & trajectories,
    const models::State & state) const;

  void integrateStateVelocities(
    xt::xtensor<float, 2> & trajectories,
    const xt::xtensor<float, 2> & state) const;

  /**
   * @brief Update control_sequence_ with state controls weighted by costs
   * using softmax function
   */
  void updateControlSequence();

  /**
   * @brief Update action_sequence_ with previous action sequence and current control
   */
  void updateActionSequence();

  /**
   * @brief Generate Batches' action sequence
   */
  void generateActionSequence();

  geometry_msgs::msg::TwistStamped
  getControlFromSequenceAsTwist(const builtin_interfaces::msg::Time & stamp);

  geometry_msgs::msg::TwistStamped
  getActionFromSequenceAsTwist(const builtin_interfaces::msg::Time & stamp);

  inline bool isHolonomic() const;

  void setOffset(double controller_frequency);

  bool fallback(bool fail);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string name_;

  std::unique_ptr<MotionModel> motion_model_;

  ParametersHandler * parameters_handler_;
  CriticManager critic_manager_;
  NoiseGenerator noise_generator_;

  models::OptimizerSettings settings_;

  models::State state_;
  models::ControlSequence control_sequence_;
  models::ActionSequence action_sequence_;
  models::Trajectories generated_trajectories_;
  models::Path path_;
  xt::xtensor<float, 1> costs_;

  CriticData critics_data_ =
  {state_, generated_trajectories_, path_, costs_, settings_.model_dt, false, nullptr,
    std::nullopt};                                                                                    /// Caution, keep references

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // MPPIC__OPTIMIZER_HPP_
