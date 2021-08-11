#include <gtest/gtest.h>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <rclcpp/executors.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "mppi/Models.hpp"
#include "mppi/Optimizer.hpp"

using namespace ultra::mppi;

using T = float;
using Optimizer = optimization::Optimizer<T>;
using nav2_costmap_2d::Costmap2D;
using rclcpp_lifecycle::LifecycleNode;
using std::shared_ptr;

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Path;

class OptimizerTest : public ::testing::Test {
protected:
  void SetUp() override {

    std::string node_name = "TestNode";
    node_ = std::make_shared<LifecycleNode>(node_name);
    costmap_ = new Costmap2D(500, 500, 0.1, 0, 0, 100);
    auto &model = models::NaiveModel<T>;

    optimizer_ = Optimizer(node_, node_name, costmap_, model);
  }

  void TearDown() override { delete costmap_; }

protected:
  shared_ptr<LifecycleNode> node_;
  Optimizer optimizer_;
  Costmap2D *costmap_;
};

template <typename T> void setDefaultHeader(T &msg) {
  msg.header.frame_id = "map";
  msg.header.stamp.nanosec = 0;
  msg.header.stamp.sec = 0;
}

PoseStamped createPose() {
  PoseStamped pose;
  setDefaultHeader(pose);
  pose.pose.position.x = 1;
  pose.pose.position.y = 1;
  pose.pose.position.z = 1;
  return pose;
}

Twist createTwist() {
  Twist twist;

  twist.linear.x = 1;
  twist.linear.y = 0;
  twist.linear.z = 0;
  return twist;
}

Path createPath() {
  Path path;
  setDefaultHeader(path);

  for (int i = 0; i < 100; i++) {
    PoseStamped p;
    setDefaultHeader(p);
    p.pose.position.x = i;
    p.pose.position.y = i * i;
    p.pose.position.z = 0;
    path.poses.push_back(p);
  }

  return path;
}

TEST_F(OptimizerTest, evalNextControlTest) {
  Twist twist;
  Path path;

  auto &&result = optimizer_.evalNextControl(twist, path);

  EXPECT_TRUE(result == TwistStamped{});
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  auto res = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return res;
}
