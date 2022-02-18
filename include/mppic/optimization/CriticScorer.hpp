#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <xtensor/xtensor.hpp>

#include "mppic/optimization/critics/CriticFunction.hpp"
#include "mppic/utils/common.hpp"
#include "mppic/utils/geometry.hpp"

namespace mppi::optimization {

// TODO pluginize
template <typename T>
class CriticScorer
{
public:
  CriticScorer() = default;
  explicit CriticScorer(std::vector<std::unique_ptr<CriticFunction<T>>> && critics)
  : critics_(std::move(critics))
  {}

  void on_configure(
    rclcpp_lifecycle::LifecycleNode * parent, const std::string & parent_name,
    const std::string & component_name, nav2_costmap_2d::Costmap2DROS * costmap_ros)
  {
    for (size_t q = 0; q < critics_.size(); q++) {
      critics_[q]->on_configure(parent, parent_name, component_name, costmap_ros);
    }
  }

  /**
   * @brief Evaluate cost for each trajectory
   *
   * @param trajectories: tensor of shape [ ..., ..., 3 ]
   * where 3 stands for x, y, yaw
   * @return Cost for each trajectory
   */
  xt::xtensor<T, 1> evalTrajectoriesScores(
    const xt::xtensor<T, 3> & trajectories, const nav_msgs::msg::Path & global_plan,
    const geometry_msgs::msg::PoseStamped & robot_pose) const
  {
    size_t trajectories_count = trajectories.shape()[0];
    xt::xtensor<T, 1> costs = xt::zeros<T>({trajectories_count});

    if (global_plan.poses.empty()) {
      return costs;
    }

    xt::xtensor<T, 2> path = std::move(geometry::toTensor<T>(global_plan));

    for (size_t q = 0; q < critics_.size(); q++) {
      critics_[q]->score(robot_pose, trajectories, path, costs);
    }

    return costs;
  }

private:
  std::vector<std::unique_ptr<CriticFunction<T>>> critics_;
};

} // namespace mppi::optimization
