// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#include <catch2/catch_all.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include "mppic/controller.hpp"
#include "mppic/optimizers/xtensor/optimizer.hpp"

#include "utils/utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

TEST_CASE("Controller doens't fail")
{
  const bool visualize = true;

  TestCostmapSettings cost_map_settings{};

  // Node Options
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params;
  setUpControllerParams(visualize, params);
  options.parameter_overrides(params);

  auto node = getDummyNode(options);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap_ros = getDummyCostmapRos(cost_map_settings);
  costmap_ros->setRobotFootprint(getDummySquareFootprint(0.01));

  auto controller = getDummyController(node, tf_buffer, costmap_ros);

  TestPose start_pose = cost_map_settings.getCenterPose();
  const double path_step = cost_map_settings.resolution;
  TestPathSettings path_settings{start_pose, 8, path_step, path_step};

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);

  controller.setPlan(path);
  controller.computeVelocityCommands(pose, velocity, {});

  REQUIRE_NOTHROW(controller.computeVelocityCommands(pose, velocity, {}));

  controller.setSpeedLimit(0.5, true);
  controller.setSpeedLimit(0.5, false);
  controller.setSpeedLimit(1.0, true);
  controller.deactivate();
  controller.cleanup();
}
