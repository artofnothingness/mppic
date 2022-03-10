#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace mppi::handlers {
using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;
using StampType = decltype(std::declval<std_msgs::msg::Header>().stamp);

class PathHandler
{
public:
  PathHandler() = default;
  ~PathHandler() = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & node_name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap, std::shared_ptr<tf2_ros::Buffer> buffer);

  void setPath(const nav_msgs::msg::Path & plan) { global_plan_ = plan; }

  nav_msgs::msg::Path & getPath() { return global_plan_; }

  /**
   * @brief transform global plan to local applying constraints,
   * then prune global_plan_
   *
   * @return global plan in local frame
   */
  nav_msgs::msg::Path transformPath(const geometry_msgs::msg::PoseStamped & robot_pose);

protected:
  bool transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  double getMaxCostmapDist();

  geometry_msgs::msg::PoseStamped
  transformToGlobalPlanFrame(const geometry_msgs::msg::PoseStamped & pose);

  nav_msgs::msg::Path
  transformPlanPosesToCostmapFrame(PathIterator begin, PathIterator end, const StampType & stamp);

  auto getGlobalPlanConsideringBounds(const geometry_msgs::msg::PoseStamped & global_pose);

  void pruneGlobalPlan(const PathIterator & end)
  {
    global_plan_.poses.erase(global_plan_.poses.begin(), end);
  }

  std::string node_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  nav_msgs::msg::Path global_plan_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPI PathHandler")};

  double lookahead_dist_{0};
  double transform_tolerance_{0};
};
} // namespace mppi::handlers
