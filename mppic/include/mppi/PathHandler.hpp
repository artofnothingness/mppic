#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "tf2_ros/buffer.h"

#include "mppi/Utils.hpp"

namespace ultra::mppi::handlers {

using std::shared_ptr;
using std::string;

using tf2_ros::Buffer;

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Path;

class PathHandler {

public:
  PathHandler() = default;
  ~PathHandler() = default;

  PathHandler(double lookagead_dist, double transform_tolerance,
              const std::shared_ptr<Buffer> &buffer)
      : lookahead_dist_(lookagead_dist),
        transform_tolerance_(transform_tolerance), tf_buffer_(buffer) {
    (void)lookahead_dist_; // TODO
  }

private:
  auto getGlobalPlanConsideringBounds(const PoseStamped &global_pose,
                                      const double &max_dist) {

    auto begin = global_plan_.poses.begin();
    auto end = global_plan_.poses.end();

    auto closest_point = std::min_element(
        begin, end,
        [&global_pose](const PoseStamped &lhs, const PoseStamped &rhs) {
          return utils::hypot(lhs, global_pose) <
                 utils::hypot(rhs, global_pose);
        });

    // Find points definitely outside of the costmap so we won't transform them.
    auto outside_costmap_point = std::find_if(
        closest_point, end, [&](const PoseStamped &global_plan_pose) {
          return utils::hypot(global_pose, global_plan_pose) > max_dist;
        });

    return std::tuple{closest_point, outside_costmap_point};
  }

public:
  void setPath(const Path &plan) { global_plan_ = plan; }
  Path& getPath() { return global_plan_; }

  Path transformPath(const PoseStamped &robot_pose,
                     const std::string &local_frame, const double &max_dist) {
    auto global_pose = transformToGlobalFrame(robot_pose);
    const auto &stamp = global_pose.header.stamp;

    auto &&[lower_bound, upper_bound] =
        getGlobalPlanConsideringBounds(global_pose, max_dist);

    auto transformed_plan = transformGlobalPlanToLocal(lower_bound, upper_bound,
                                                       stamp, local_frame);

    pruneGlobalPlan(lower_bound);

    if (transformed_plan.poses.empty())
      throw std::runtime_error("Resulting plan has 0 poses in it.");

    return transformed_plan;
  }

private:
  template <typename Iter, typename Stamp>
  Path transformGlobalPlanToLocal(Iter begin, Iter end, const Stamp &stamp,
                                  const string &frame) {

    auto transform_pose = [&](const auto &global_plan_pose) {
      PoseStamped global_pose;
      PoseStamped local_pose;

      global_pose.header.frame_id = global_plan_.header.frame_id;
      global_pose.header.stamp = stamp;
      global_pose.pose = global_plan_pose.pose;

      transformPose(frame, global_pose, local_pose);
      return local_pose;
    };

    Path plan;
    std::transform(begin, end, std::back_inserter(plan.poses), transform_pose);
    plan.header.frame_id = frame;
    plan.header.stamp = stamp;

    return plan;
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
      RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
    }
    return false;
  }

  template <typename T> void pruneGlobalPlan(const T &end) {
    global_plan_.poses.erase(global_plan_.poses.begin(), end);
  }

  auto transformToGlobalFrame(const PoseStamped &pose) -> PoseStamped {
    if (global_plan_.poses.empty()) {
      throw std::runtime_error("Received plan with zero length");
    }

    PoseStamped robot_pose;
    if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
      throw std::runtime_error(
          "Unable to transform robot pose into global plan's frame");
    }

    return robot_pose;
  }

private:
  double lookahead_dist_;
  double transform_tolerance_;
  rclcpp::Logger logger_{rclcpp::get_logger("PathHandler")};

  Path global_plan_;
  shared_ptr<Buffer> tf_buffer_;
};

} // namespace ultra::mppi::handlers
