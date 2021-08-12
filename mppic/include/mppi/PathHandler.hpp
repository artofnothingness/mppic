#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

#include "utils/common.hpp"
#include "utils/geometry.hpp"

namespace mppi::handlers {

class PathHandler {

public:
  PathHandler() = default;
  ~PathHandler() = default;

  PathHandler(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
              const std::string &node_name,
              const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap,
              const std::shared_ptr<tf2_ros::Buffer> &buffer) {

    node_name_ = node_name;
    tf_buffer_ = buffer;
    parent_ = parent;
    costmap_ = costmap;
  }

  void on_configure() {
    getParams();
    RCLCPP_INFO(logger_, "Configured");
  }
  void on_cleanup() {}
  void on_activate() {}
  void on_deactivate() {}

private:
  void getParams() {
    auto getParam = [&](const std::string &param_name, auto default_value) {
      std::string name = node_name_ + '.' + param_name;
      return utils::getParam(name, default_value, parent_);
    };

    lookahead_dist_ = getParam("lookahead_dist", 1.2);
    transform_tolerance_ = getParam("transform_tolerance", 1.2);
  }

  auto getGlobalPlanConsideringBounds(
      const geometry_msgs::msg::PoseStamped &global_pose) {

    auto begin = global_plan_.poses.begin();
    auto end = global_plan_.poses.end();

    auto closest_point = std::min_element(
        begin, end,
        [&global_pose](const geometry_msgs::msg::PoseStamped &lhs,
                       const geometry_msgs::msg::PoseStamped &rhs) {
          return geometry::hypot(lhs, global_pose) <
                 geometry::hypot(rhs, global_pose);
        });

    auto max_costmap_dist = getMaxCostmapDist();
    auto last_point = std::find_if(
        closest_point, end,
        [&](const geometry_msgs::msg::PoseStamped &global_plan_pose) {
          auto &&dist = geometry::hypot(global_pose, global_plan_pose);
          return dist > max_costmap_dist or dist > lookahead_dist_;
        });

    return std::tuple{closest_point, last_point};
  }

public:
  void setPath(const nav_msgs::msg::Path &plan) { global_plan_ = plan; }
  nav_msgs::msg::Path &getPath() { return global_plan_; }

  nav_msgs::msg::Path
  transformPath(const geometry_msgs::msg::PoseStamped &robot_pose) {
    auto global_pose = transformToGlobalFrame(robot_pose);
    const auto &stamp = global_pose.header.stamp;

    auto &&[lower_bound, upper_bound] =
        getGlobalPlanConsideringBounds(global_pose);

    auto transformed_plan =
        transformGlobalPlanToLocal(lower_bound, upper_bound, stamp);

    pruneGlobalPlan(lower_bound);

    if (transformed_plan.poses.empty())
      throw std::runtime_error("Resulting plan has 0 poses in it.");

    return transformed_plan;
  }

private:
  template <typename Iter, typename Stamp>
  nav_msgs::msg::Path transformGlobalPlanToLocal(Iter begin, Iter end,
                                                 const Stamp &stamp) {

    auto base_frame = costmap_->getBaseFrameID();

    auto transform_pose = [&](const auto &global_plan_pose) {
      geometry_msgs::msg::PoseStamped global_pose;
      geometry_msgs::msg::PoseStamped local_pose;

      global_pose.header.frame_id = global_plan_.header.frame_id;
      global_pose.header.stamp = stamp;
      global_pose.pose = global_plan_pose.pose;

      transformPose(base_frame, global_pose, local_pose);
      return local_pose;
    };

    nav_msgs::msg::Path plan;
    std::transform(begin, end, std::back_inserter(plan.poses), transform_pose);
    plan.header.frame_id = base_frame;
    plan.header.stamp = stamp;

    return plan;
  }

  bool transformPose(const std::string &frame,
                     const geometry_msgs::msg::PoseStamped &in_pose,
                     geometry_msgs::msg::PoseStamped &out_pose) const {

    if (in_pose.header.frame_id == frame) {
      out_pose = in_pose;
      return true;
    }

    try {
      tf_buffer_->transform(in_pose, out_pose, frame,
                            tf2::durationFromSec(transform_tolerance_));
      out_pose.header.frame_id = frame;
      return true;
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
    }
    return false;
  }

  template <typename T> void pruneGlobalPlan(const T &end) {
    global_plan_.poses.erase(global_plan_.poses.begin(), end);
  }

  double getMaxCostmapDist() {
    const auto &costmap = costmap_->getCostmap();
    return std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
           costmap->getResolution() / 2.0;
  }

  auto transformToGlobalFrame(const geometry_msgs::msg::PoseStamped &pose)
      -> geometry_msgs::msg::PoseStamped {
    if (global_plan_.poses.empty()) {
      throw std::runtime_error("Received plan with zero length");
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
      throw std::runtime_error(
          "Unable to transform robot pose into global plan's frame");
    }

    return robot_pose;
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
