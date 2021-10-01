#pragma once

#include "mppi/PathHandler.hpp"

namespace mppi::handlers {

template<typename Iter, typename Stamp>
auto PathHandler::transformGlobalPlan(
  Iter begin,
  Iter end,
  const Stamp &stamp,
  const std::string &frame)
  -> nav_msgs::msg::Path
{
  auto transformToFrame = [&](const auto &global_plan_pose) {
    geometry_msgs::msg::PoseStamped global_pose;
    geometry_msgs::msg::PoseStamped local_pose;

    global_pose.header.frame_id = global_plan_.header.frame_id;
    global_pose.header.stamp = stamp;
    global_pose.pose = global_plan_pose.pose;

    transformPose(frame, global_pose, local_pose);
    return local_pose;
  };

  nav_msgs::msg::Path plan;

  std::transform(begin, end, std::back_inserter(plan.poses), transformToFrame);
  plan.header.frame_id = frame;
  plan.header.stamp = stamp;

  return plan;
}

}// namespace mppi::handlers
