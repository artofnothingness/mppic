#pragma once

#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace mppi::handlers {

class PathHandler {
public:
  PathHandler() = default;
  ~PathHandler() = default;

  void on_configure(rclcpp_lifecycle::LifecycleNode *const parent, const std::string &node_name,
                    nav2_costmap_2d::Costmap2DROS *const costmap, tf2_ros::Buffer *const buffer);

  void on_cleanup();
  void on_activate();
  void on_deactivate();

  void
  setPath(const nav_msgs::msg::Path &plan) {
    global_plan_ = plan;
  }

  nav_msgs::msg::Path &
  getPath() {
    return global_plan_;
  }

  /**
   * @brief transform global plan to local applying constraints,
   * then prune global_plan_
   *
   * @return global plan in local frame
   */
  nav_msgs::msg::Path transformPath(const geometry_msgs::msg::PoseStamped &robot_pose);

private:
  void getParams();

  bool transformPose(const std::string &frame, const geometry_msgs::msg::PoseStamped &in_pose,
                     geometry_msgs::msg::PoseStamped &out_pose) const;

  double getMaxCostmapDist();

  geometry_msgs::msg::PoseStamped transformToGlobalFrame(
      const geometry_msgs::msg::PoseStamped &pose);

  template <typename Iter, typename Stamp>
  nav_msgs::msg::Path transformGlobalPlan(Iter begin, Iter end, const Stamp &stamp,
                                          const std::string &frame);

  auto getGlobalPlanConsideringBounds(const geometry_msgs::msg::PoseStamped &global_pose);

  template <typename T>
  void
  pruneGlobalPlan(const T &end) {
    global_plan_.poses.erase(global_plan_.poses.begin(), end);
  }

  rclcpp_lifecycle::LifecycleNode *parent_;
  std::string node_name_;
  nav2_costmap_2d::Costmap2DROS *costmap_;
  tf2_ros::Buffer *tf_buffer_;

  nav_msgs::msg::Path global_plan_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPI PathHandler")};

  double lookahead_dist_;
  double transform_tolerance_;
};

}  // namespace mppi::handlers
