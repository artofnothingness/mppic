// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#include <catch2/catch_all.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_core/goal_checker.hpp>

#include "mppic/path_handler.hpp"

#include "utils/utils.hpp"

using namespace std::chrono_literals;

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;


TEST_CASE("Path handler doesn't fail")
{
  unsigned int path_points = 50u;

  TestCostmapSettings cost_map_settings{};
  auto costmap_ros = getDummyCostmapRos(cost_map_settings);

  TestPose start_pose = cost_map_settings.getCenterPose();
  double path_step = cost_map_settings.resolution;
  TestPathSettings path_settings{start_pose, path_points, path_step, path_step};

  auto node = getDummyNode(rclcpp::NodeOptions{});

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  tf_buffer->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  auto tf_broadcaster_ =
    std::make_shared<tf2_ros::TransformBroadcaster>(node);
  std::atomic_bool stop_flag = false;

  auto map_odom_broadcaster = std::async(
      std::launch::async, sendTf, "map", "odom", tf_broadcaster_, node, 
      std::ref(stop_flag));
  auto odom_base_link_broadcaster = std::async(
    std::launch::async, sendTf, "odom", "base_link", tf_broadcaster_, node, 
      std::ref(stop_flag));

  auto parameters_handler = std::make_unique<mppi::ParametersHandler>(node);
  auto path_handler = getDummyPathHandler(node, costmap_ros, tf_buffer, parameters_handler.get());

  auto path = getIncrementalDummyPath(node, path_settings);
  geometry_msgs::msg::PoseStamped pose;

  path_handler.setPath(path);
#ifdef DO_BENCHMARKS
  BENCHMARK("evalControl Benchmark") {
    pose = getDummyPointStamped(node, start_pose);
    return path_handler.transformPath(pose);
  };
#else
  REQUIRE_NOTHROW(path_handler.transformPath(pose));
#endif
  stop_flag = true;
  map_odom_broadcaster.wait();
  odom_base_link_broadcaster.wait();
}
