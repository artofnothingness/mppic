#pragma once

#include <pluginlib/class_loader.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <xtensor/xtensor.hpp>

#include "mppic/optimization/scoring/CriticFunction.hpp"
#include "mppic/utils/common.hpp"
#include "mppic/utils/geometry.hpp"

namespace mppi::optimization {

template <typename T>
class CriticScorer
{
public:
  CriticScorer() = default;

  void on_configure(
    rclcpp_lifecycle::LifecycleNode * parent, const std::string & node_name,
    nav2_costmap_2d::Costmap2DROS * costmap_ros)
  {
    parent_ = parent;
    costmap_ros_ = costmap_ros;
    node_name_ = node_name;

    getParams();
    setLoader();
    loadCritics();
    configureCritics();

    RCLCPP_INFO(logger_, "Configured");
  }

  void setLoader()
  {
    if (!loader_) {
      loader_ = std::make_unique<pluginlib::ClassLoader<optimization::CriticFunction<T>>>(
        "mppic", getFullName(base_name_));
    }
  }

  std::string getFullName(const std::string & name)
  {
    return "mppi::optimization::" + name + "<" + critics_type_ + ">";
  }

  void getParams()
  {
    auto getParam = utils::getParamGetter(parent_, node_name_);
    getParam(critics_names_, "critics_names", std::vector<std::string>{});
    getParam(critics_type_, "critics_type", std::string("float"));
  }

  void loadCritics()
  {
    critics_.clear();
    for (auto name : critics_names_) {
      std::string fullname = getFullName(name);
      auto instance = std::unique_ptr<optimization::CriticFunction<T>>(
        loader_->createUnmanagedInstance(fullname));
      critics_.push_back(std::move(instance));
      RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
    }
  }

  void configureCritics()
  {
    for (size_t q = 0; q < critics_.size(); q++) {
      critics_[q]->on_configure(parent_, node_name_ + "." + critics_names_[q], costmap_ros_);
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
  rclcpp_lifecycle::LifecycleNode * parent_{nullptr};
  nav2_costmap_2d::Costmap2DROS * costmap_ros_{nullptr};
  std::string node_name_;

  std::vector<std::string> critics_names_;
  std::string critics_type_;
  const std::string base_name_ = "CriticFunction";

  std::unique_ptr<pluginlib::ClassLoader<optimization::CriticFunction<T>>> loader_;
  std::vector<std::unique_ptr<CriticFunction<T>>> critics_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPI CriticScorer")};
};

} // namespace mppi::optimization
