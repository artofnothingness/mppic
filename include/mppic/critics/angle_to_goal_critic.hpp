#pragma once

#include <xtensor/xnorm.hpp>
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "mppic/critic_function.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

class AngleToGoalCritic : public CriticFunction
{
public:

  void initialize() override
  {
    auto node = parent_.lock();

    auto getParam = utils::getParamGetter(node, node_name_);
    getParam(power_, "angle_to_goal_cost_power", 1);
    getParam(weight_, "angle_to_goal_cost_weight", 15);
    RCLCPP_INFO(
      logger_, "AngleToGoalCritic instantiated with %d power and %f weight.", power_, weight_);
  }

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  virtual void score(
    const geometry_msgs::msg::PoseStamped & robot_pose, const xt::xtensor<double, 3> & trajectories,
    const xt::xtensor<double, 2> & path, xt::xtensor<double, 1> & costs) override
  {
    auto init_yaw = tf2::getYaw(robot_pose.pose.orientation);

    auto goal_x = xt::view(path, -1, 0);
    auto goal_y = xt::view(path, -1, 1);
    auto traj_xs = xt::view(trajectories, xt::all(), xt::all(), 0);
    auto traj_ys = xt::view(trajectories, xt::all(), xt::all(), 1);
    auto traj_yaws = xt::view(trajectories, xt::all(), xt::all(), 2);

    auto yaws_between_points = atan2(goal_y - traj_ys, goal_x - traj_xs);
    auto yaws = xt::abs(traj_yaws - yaws_between_points);

    costs += xt::pow(xt::mean(yaws, {1}) * weight_, power_);
  }

protected:
  unsigned int power_{0};
  double weight_{0};
};

} // namespace mppi::critics
