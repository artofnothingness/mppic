#include "mppic/Controller.hpp"

#include "mppic/StateModels.hpp"
#include "mppic/utils/common.hpp"
#include "mppic/utils/geometry.hpp"

namespace mppi {
template<typename T>
void Controller<T>::configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
  std::string node_name,
  const std::shared_ptr<tf2_ros::Buffer> &tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros)
{
  parent_ = parent.get();
  costmap_ros_ = costmap_ros.get();
  tf_buffer_ = tf.get();
  node_name_ = std::move(node_name);

  getParams();
  setPublishers();
  configureComponents();
}

template<typename T>
void Controller<T>::cleanup()
{
  optimizer_.on_cleanup();
  path_handler_.on_cleanup();
  trajectory_visualizer_.on_cleanup();
  transformed_path_pub_.reset();
}

template<typename T>
void Controller<T>::activate()
{
  transformed_path_pub_->on_activate();
  optimizer_.on_activate();
  path_handler_.on_activate();
  trajectory_visualizer_.on_activate();
}

template<typename T>
void Controller<T>::deactivate()
{
  transformed_path_pub_->on_deactivate();
  optimizer_.on_deactivate();
  path_handler_.on_deactivate();
  trajectory_visualizer_.on_deactivate();
}

template<typename T>
geometry_msgs::msg::TwistStamped Controller<T>::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped &robot_pose,
  const geometry_msgs::msg::Twist &robot_speed)
{
  auto &&transformed_plan = path_handler_.transformPath(robot_pose);
  auto &&cmd = optimizer_.evalControl(robot_pose, robot_speed, transformed_plan);

  if (visualize_) {
    auto &&plan_ptr = std::make_unique<nav_msgs::msg::Path>(std::move(transformed_plan));
    handleVisualizations(robot_pose, robot_speed, std::move(plan_ptr));
  }

  return cmd;
}

template<typename T>
void Controller<T>::handleVisualizations(const geometry_msgs::msg::PoseStamped &robot_pose,
  const geometry_msgs::msg::Twist &robot_speed,
  std::unique_ptr<nav_msgs::msg::Path> &&transformed_plan)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), 5, 2);
  trajectory_visualizer_.add(optimizer_.evalTrajectoryFromControlSequence(robot_pose, robot_speed));
  trajectory_visualizer_.visualize();

  transformed_path_pub_->publish(std::move(transformed_plan));
}

template<typename T>
void Controller<T>::getParams()
{
  auto getParam = utils::getParamGetter(parent_, node_name_);

  getParam(visualize_, "visualize", true);
}

template<typename T>
void Controller<T>::setPublishers()
{
  transformed_path_pub_ =
    parent_->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
}

template<typename T>
void Controller<T>::configureComponents()
{
  auto &model = models::NaiveModel<T>;

  optimizer_.on_configure(parent_, node_name_, costmap_ros_, model);
  path_handler_.on_configure(parent_, node_name_, costmap_ros_, tf_buffer_);
  trajectory_visualizer_.on_configure(parent_, costmap_ros_->getGlobalFrameID());
}

}// namespace mppi

#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

template class mppi::Controller<float>;

PLUGINLIB_EXPORT_CLASS(mppi::Controller<float>, nav2_core::Controller)
