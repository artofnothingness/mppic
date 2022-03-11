#pragma once

#include <xtensor/xmath.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "mppic/critic_function.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

class GoalCritic : public CriticFunction
{
public:

  void initialize() override
  {
    auto node = parent_.lock();
    auto getParam = utils::getParamGetter(node, name_);
    
    getParam(power_, "goal_cost_power", 1);
    getParam(weight_, "goal_cost_weight", 20);
    RCLCPP_INFO(
      logger_, "GoalCritic instantiated with %d power and %f weight.", power_, weight_);
  }

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  virtual void score(
    const geometry_msgs::msg::PoseStamped & /*robot_pose*/, const xt::xtensor<double, 3> & trajectories,
    const xt::xtensor<double, 2> & path, xt::xtensor<double, 1> & costs) override
  {
    const auto goal_points = xt::view(path, -1, xt::range(0, 2));

    auto trajectories_end = xt::view(trajectories, xt::all(), -1, xt::range(0, 2));

    auto dim = trajectories_end.dimension() - 1;

    auto && dists_trajectories_end_to_goal =
      xt::norm_l2(std::move(trajectories_end) - goal_points, {dim});

    costs += xt::pow(std::move(dists_trajectories_end_to_goal) * weight_, power_);
  }

protected:
  unsigned int power_{0};
  double weight_{0};
};

} // namespace mppi::critics
