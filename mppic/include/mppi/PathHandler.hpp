#pragma once

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace mppi::handlers {

class PathHandler {

public:
  PathHandler() = default;
  ~PathHandler() = default;

  PathHandler(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
              const std::string &node_name,
              const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap,
              const std::shared_ptr<tf2_ros::Buffer> &buffer)
      : parent_(parent), node_name_(node_name), costmap_(costmap), tf_buffer_(buffer) {}

  void on_configure();
  void on_cleanup();
  void on_activate();
  void on_deactivate();

  void setPath(const nav_msgs::msg::Path &plan) { global_plan_ = plan; }
  auto getPath() -> nav_msgs::msg::Path & { return global_plan_; }

  /**
   * @brief transform global plan to local applying constraints,
   * then prune global_plan_
   *
   * @return global plan in local frame
   */
  auto transformPath(const geometry_msgs::msg::PoseStamped &robot_pose) 
  -> nav_msgs::msg::Path;

private:
  auto getParams()
  -> void;

  auto 
  transformPose(
      const std::string &frame,
      const geometry_msgs::msg::PoseStamped &in_pose,
      geometry_msgs::msg::PoseStamped &out_pose
  ) const
  -> bool;

  double getMaxCostmapDist();

  auto transformToGlobalFrame(const geometry_msgs::msg::PoseStamped &pose)
  -> geometry_msgs::msg::PoseStamped;

  template <typename Iter, typename Stamp>
  auto transformGlobalPlan(Iter begin, Iter end, const Stamp &stamp, 
                           const std::string &frame)
  -> nav_msgs::msg::Path;

  auto getGlobalPlanConsideringBounds(const geometry_msgs::msg::PoseStamped &global_pose);

  template <typename T>
  auto pruneGlobalPlan(const T &end) 
  -> void 
  {
    global_plan_.poses.erase(global_plan_.poses.begin(), end);
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::string node_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  double lookahead_dist_;
  double transform_tolerance_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPI PathHandler")};

  nav_msgs::msg::Path global_plan_;
};

} // namespace mppi::handlers
