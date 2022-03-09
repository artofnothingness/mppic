#include "mppic/Controller.hpp"
#include "mppic/optimization/StateModels.hpp"
#include "mppic/utils.hpp"

namespace mppi {

void Controller::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string node_name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  // Set inputs
  parent_ = parent.lock().get();
  costmap_ros_ = costmap_ros.get();
  tf_buffer_ = tf.get();
  node_name_ = std::move(node_name);

  // Get high level controller parameters
  auto getParam = utils::getParamGetter(parent_, node_name_);
  getParam(visualize_, "visualize", true);

  // Configure composed objects
  auto & model = optimization::models::NaiveModel;
  optimizer_.initialize(parent_, node_name_, costmap_ros_, model);
  path_handler_.initialize(parent_, node_name_, costmap_ros_, tf_buffer_);
  trajectory_visualizer_.on_configure(parent_, costmap_ros_->getGlobalFrameID());
}

void Controller::cleanup()
{
  trajectory_visualizer_.on_cleanup();
}

void Controller::activate()
{
  trajectory_visualizer_.on_activate();
}

void Controller::deactivate()
{
  trajectory_visualizer_.on_deactivate();
}

geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);
  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan);

  if (visualize_) {
    handleVisualizations(robot_pose, robot_speed, transformed_plan);
  }

  return cmd;
}

void Controller::handleVisualizations(
  const geometry_msgs::msg::PoseStamped & robot_pose, const geometry_msgs::msg::Twist & robot_speed,
  nav_msgs::msg::Path & transformed_plan)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), 5, 2);
  trajectory_visualizer_.add(optimizer_.evalTrajectoryFromControlSequence(robot_pose, robot_speed));
  trajectory_visualizer_.visualize(transformed_plan);
}

void Controller::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

void Controller::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  RCLCPP_ERROR(parent_->get_logger(), "MPPI's dynamic speed limit adjustment callback is not yet implemented!");
}

} // namespace mppi

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mppi::Controller, nav2_core::Controller)
