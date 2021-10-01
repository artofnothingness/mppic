#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "utils/common.hpp"
#include "utils/geometry.hpp"

#include "mppi/impl/PathHandler.hpp"

namespace mppi::handlers {

void PathHandler::on_configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
  const std::string &node_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap,
  const std::shared_ptr<tf2_ros::Buffer> &buffer)
{

  parent_ = parent;
  node_name_ = node_name;
  costmap_ = costmap;
  tf_buffer_ = buffer;

  getParams();
  RCLCPP_INFO(logger_, "Configured");
}
void PathHandler::on_cleanup() {}
void PathHandler::on_activate() {}
void PathHandler::on_deactivate() {}

auto PathHandler::getGlobalPlanConsideringBounds(
  const geometry_msgs::msg::PoseStamped &global_pose)
{

  auto begin = global_plan_.poses.begin();
  auto end = global_plan_.poses.end();

  auto closest_point = std::min_element(
    begin, end, [&global_pose](const geometry_msgs::msg::PoseStamped &lhs, const geometry_msgs::msg::PoseStamped &rhs) {
      return geometry::hypot(lhs, global_pose) < geometry::hypot(rhs, global_pose);
    });

  auto max_costmap_dist = getMaxCostmapDist();
  auto last_point = std::find_if(
    closest_point, end, [&](const geometry_msgs::msg::PoseStamped &global_plan_pose) {
      auto &&dist = geometry::hypot(global_pose, global_plan_pose);
      return dist > max_costmap_dist or dist > lookahead_dist_;
    });

  return std::tuple{ closest_point, last_point };
}

void PathHandler::getParams()
{
  auto getParam = [&](const std::string &param_name, auto default_value) {
    std::string name = node_name_ + '.' + param_name;
    return utils::getParam(name, default_value, parent_);
  };

  lookahead_dist_ = getParam("lookahead_dist", 1.0);
  transform_tolerance_ = getParam("transform_tolerance", 1);
}

auto PathHandler::transformToGlobalFrame(
  const geometry_msgs::msg::PoseStamped &pose)
  -> geometry_msgs::msg::PoseStamped
{

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

nav_msgs::msg::Path
  PathHandler::transformPath(const geometry_msgs::msg::PoseStamped &robot_pose)
{

  auto global_pose = transformToGlobalFrame(robot_pose);
  const auto &stamp = global_pose.header.stamp;

  auto &&[lower_bound, upper_bound] =
    getGlobalPlanConsideringBounds(global_pose);

  auto transformed_plan =
    transformGlobalPlan(lower_bound, upper_bound, stamp, costmap_->getGlobalFrameID());

  pruneGlobalPlan(lower_bound);

  if (transformed_plan.poses.empty()) {
    throw std::runtime_error("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PathHandler::transformPose(
  const std::string &frame,
  const geometry_msgs::msg::PoseStamped &in_pose,
  geometry_msgs::msg::PoseStamped &out_pose) const
{

  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_buffer_->transform(
      in_pose, out_pose, frame, tf2::durationFromSec(transform_tolerance_));
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double PathHandler::getMaxCostmapDist()
{
  const auto &costmap = costmap_->getCostmap();
  return std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) * costmap->getResolution() / 2.0;
}

}// namespace mppi::handlers
