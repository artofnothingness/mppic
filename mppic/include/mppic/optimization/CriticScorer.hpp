#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <xtensor/xtensor.hpp>

#include "mppic/utils/common.hpp"
#include "mppic/utils/geometry.hpp"
#include "mppic_base/CriticFunction.hpp"

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
    rclcpp_lifecycle::LifecycleNode * parent, const std::string & node_name,
    nav2_costmap_2d::Costmap2DROS * costmap_ros)
  {

    auto getParam = utils::getParamGetter(parent, node_name);

    std::vector<std::string> critics_names;
    std::string critics_type;
    getParam(critics_names, "critics_names", std::vector<std::string>{});
    getParam(critics_type, "critics_type", std::string("float"));

    auto getFullName = [&critics_type](const std::string & name) {
      return "mppi::optimization::" + name + "<" + critics_type + ">";
    };

    for (auto name : critics_names) {
      getFullName(name);
    }

    const std::string base_name = "CriticFunction";
    pluginlib::ClassLoader<optimization::CriticFunction<T>> loader(
      "mppic_base", getFullName(base_name));

    critics_.clear();
    for (auto name : critics_names) {
      std::string fullname = getFullName(name);
      auto instance =
        std::unique_ptr<optimization::CriticFunction<T>>(loader.createUnmanagedInstance(fullname));
      critics_.push_back(std::move(instance));
      RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
    }

    for (size_t q = 0; q < critics_.size(); q++) {
      critics_[q]->on_configure(parent, node_name + "." + critics_names[q], costmap_ros);
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
  rclcpp::Logger logger_{rclcpp::get_logger("MPPI CriticScorer")};
};

} // namespace mppi::optimization
