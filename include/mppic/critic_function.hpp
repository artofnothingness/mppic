#ifndef MPPIC__CRITIC_FUNCTION_HPP_
#define MPPIC__CRITIC_FUNCTION_HPP_

#include <string>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace mppi::critics
{
class CriticFunction
{
public:
  CriticFunction() = default;
  virtual ~CriticFunction() = default;

  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, 
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    parent_ = parent;
    logger_ = parent_.lock()->get_logger();
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    initialize();
  }

  virtual void initialize() = 0;

  virtual void score(
    const geometry_msgs::msg::PoseStamped & robot_pose, const xt::xtensor<double, 3> & trajectories,
    const xt::xtensor<double, 2> & path, xt::xtensor<double, 1> & costs) = 0;

protected:
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

} // namespace mppi::critics

#endif  // MPPIC__CRITIC_FUNCTION_HPP_
