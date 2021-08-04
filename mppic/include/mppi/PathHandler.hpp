#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "mppi/Utils.hpp"

namespace ultra::mppi::handlers {

using std::shared_ptr;
using std::string;

using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::Costmap2DROS;
using rclcpp_lifecycle::LifecycleNode;

using tf2_ros::Buffer;

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Path;

class PathHandler {

public:
  PathHandler() = default;
  ~PathHandler() = default;

  PathHandler(const shared_ptr<LifecycleNode> &parent, const string &node_name,
              const shared_ptr<Buffer> &tf,
              const shared_ptr<Costmap2DROS> &costmap_ros) {

    parent_ = parent;
    costmap_ros_ = costmap_ros;
    tf_buffer_ = tf;

    using namespace utils;
    getParam(node_name + ".lookahead_dist", 1.2, parent_, lookahead_dist_);
    getParam(node_name + ".transform_tolerance", 1.2, parent_,
             transform_tolerance_);
  }

private:
  auto getGlobalPlanConsideringBounds(const PoseStamped &global_pose,
                                      double max_transform_dist) {

    auto begin = global_plan_.poses.begin();
    auto end = global_plan_.poses.end();

    auto closes_point = std::min_element(
        begin, end,
        [&global_pose](const PoseStamped &lhs, const PoseStamped &rhs) {
          return utils::hypot(lhs, global_pose) <
                 utils::hypot(rhs, global_pose);
        });

    // Find points definitely outside of the costmap so we won't transform them.
    auto outside_costmap_point = std::find_if(
        closes_point, end, [&](const PoseStamped &global_plan_pose) {
          return utils::hypot(global_pose, global_plan_pose) >
                 max_transform_dist;
        });

    return std::tuple{closes_point, outside_costmap_point};
  }

public:
  Path transformPath(const PoseStamped &robot_pose) {
    auto global_pose = transformToGlobalFrame(robot_pose);
    const auto &stamp = global_pose.header.stamp;
    double max_dist = getMaxTransformDistance();

    auto &&[lower_bound, upper_bound] =
        getGlobalPlanConsideringBounds(global_pose, max_dist);

    auto transformed_plan =
        transformGlobalPlanToLocal(lower_bound, upper_bound, stamp);

    pruneGlobalPlan(lower_bound);

    /* global_path_pub_->publish(transformed_plan); */ // TODO

    if (transformed_plan.poses.empty())
      throw std::runtime_error("Resulting plan has 0 poses in it.");

    return transformed_plan;
  }

  void setPlan(const Path &plan) { global_plan_ = plan; }

private:
  template <typename Iter, typename Stamp>
  Path transformGlobalPlanToLocal(Iter begin, Iter end, const Stamp &stamp) {

    auto transform_pose = [&](const auto &global_plan_pose) {
      PoseStamped global_pose;
      PoseStamped local_pose;

      global_pose.header.frame_id = global_plan_.header.frame_id;
      global_pose.header.stamp = stamp;
      global_pose.pose = global_plan_pose.pose;

      transformPose(costmap_ros_->getBaseFrameID(), global_pose, local_pose);
      return local_pose;
    };

    Path plan;
    std::transform(begin, end, std::back_inserter(plan.poses), transform_pose);
    plan.header.frame_id = costmap_ros_->getBaseFrameID();
    plan.header.stamp = stamp;

    return plan;
  }

  void pruneGlobalPlan(const auto &end) {
    global_plan_.poses.erase(global_plan_.poses.begin(), end);
  }

  auto transformToGlobalFrame(const PoseStamped &pose) -> PoseStamped {
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

  bool transformPose(const string &frame, const PoseStamped &in_pose,
                     PoseStamped &out_pose) const {

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
      RCLCPP_ERROR(parent_->get_logger(), "Exception in transformPose: %s",
                   ex.what());
    }
    return false;
  }

  double getMaxTransformDistance() {
    Costmap2D *costmap = costmap_ros_->getCostmap();

    return std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
           costmap->getResolution() / 2.0;
  }

private:
  shared_ptr<LifecycleNode> parent_;
  shared_ptr<Buffer> tf_buffer_;
  shared_ptr<Costmap2DROS> costmap_ros_;

  Path global_plan_;

  double lookahead_dist_;
  double transform_tolerance_;
};

} // namespace ultra::mppi::handlers
