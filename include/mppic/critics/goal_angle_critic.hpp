#pragma once

#include <xtensor/xnorm.hpp>
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "mppic/critic_function.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics {

template <typename T>
class GoalAngleCritic : public CriticFunction<T>
{
public:
  using CriticFunction<T>::parent_;
  using CriticFunction<T>::node_name_;

  void getParams() override
  {
    auto node = parent_.lock();
    auto getParam = utils::getParamGetter(node, node_name_);
    getParam(power_, "goal_angle_cost_power", 1);
    getParam(weight_, "goal_angle_cost_weight", 15);
    getParam(threshold_to_consider_goal_angle_, "threshold_to_consider_goal_angle", 0.30);
  }

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  virtual void score(
    const geometry_msgs::msg::PoseStamped & robot_pose, const xt::xtensor<T, 3> & trajectories,
    const xt::xtensor<T, 2> & path, xt::xtensor<T, 1> & costs) override
  {
    xt::xtensor<T, 1> tensor_pose = {
      static_cast<T>(robot_pose.pose.position.x), static_cast<T>(robot_pose.pose.position.y)};

    auto path_points = xt::view(path, -1, xt::range(0, 2));

    double points_to_goal_dists = xt::norm_l2(tensor_pose - path_points, {0})();

    if (points_to_goal_dists < threshold_to_consider_goal_angle_) {
      auto yaws = xt::view(trajectories, xt::all(), xt::all(), 2);
      auto goal_yaw = xt::view(path, -1, 2);

      costs += xt::pow(xt::mean(xt::abs(yaws - goal_yaw), {1}) * weight_, power_);
    }
  }

protected:
  double threshold_to_consider_goal_angle_{0};
  unsigned int power_{0};
  double weight_{0};
};

} // namespace mppi::critics
