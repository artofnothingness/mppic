#pragma once

#include "nav2_core/controller.hpp"

#include "rclcpp/rclcpp.hpp"

#include <string>

#include "mppi/Optimizer.hpp"
#include "mppi/Utils.hpp"


namespace ultra::mppi {

template<typename T>
class Controller : public nav2_core::Controller {

  using Optimizer = optimization::Optimizer<T>;

  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Twist = geometry_msgs::msg::Twist;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Path = nav_msgs::msg::Path;
  using Costmap2DROS = nav2_costmap_2d::Costmap2DROS;
  using TfBuffer = tf2_ros::Buffer;
  using ManagedNode = rclcpp_lifecycle::LifecycleNode;

public:
  Controller() = default;
  ~Controller() override = default;

  void configure(ManagedNode::SharedPtr const &parent,
                 std::string node_name, 
                 std::shared_ptr<TfBuffer> const &tf,
                 std::shared_ptr<Costmap2DROS> const &costmap_ros) override {


    m_costmap_ros_ = costmap_ros;
    m_tf_ = tf;
    m_node_name_ = node_name;
    m_parent = parent;

    m_optimizer_ = Optimizer();
    m_optimizer_.configure(parent);
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  auto computeVelocityCommands(PoseStamped const &pose, Twist const &velocity)
  -> TwistStamped override {

    (void)velocity;
    (void)pose;
    return TwistStamped{};
  }

  void setPlan(Path const &path) override { m_global_plan_ = path; }

  rclcpp_lifecycle::LifecycleNode::SharedPtr m_parent;
private:

  // Configure step params
  std::shared_ptr<tf2_ros::Buffer> m_tf_;
  std::string m_node_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> m_costmap_ros_;

  Path m_global_plan_;

  // Entities
  Optimizer m_optimizer_;
};

} // namespace ultra::mppi
