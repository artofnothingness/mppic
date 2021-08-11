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
#include <benchmark/benchmark.h>

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

class OptimizerBenchmark : public benchmark::Fixture {
public:
  void SetUp(const ::benchmark::State &state) {
    (void)state;

    std::string node_name = "BenchmarkNode";
    node_ = std::make_shared<LifecycleNode>(node_name);
    costmap_ = new Costmap2D(500, 500, 0.1, 0, 0, 100);
    auto &model = models::NaiveModel<T>;

    optimizer_ = Optimizer(node_, node_name, costmap_, model);
    optimizer_.on_configure();
    optimizer_.on_activate();
  }

  void TearDown(const ::benchmark::State &state) {
    (void)state;
    optimizer_.on_cleanup();
    node_.reset();
    delete costmap_;
  }

protected:
  shared_ptr<LifecycleNode> node_;
  Optimizer optimizer_;
  Costmap2D *costmap_;
};

BENCHMARK_DEFINE_F(OptimizerBenchmark, evalNextControlBenchmark)
(benchmark::State &st) {
  for (auto _ : st) {
    Twist twist = createTwist();
    Path path = createPath();
    auto result = optimizer_.evalNextControl(twist, path);
  }
}

BENCHMARK_REGISTER_F(OptimizerBenchmark, evalNextControlBenchmark)
    ->Unit(benchmark::kMillisecond);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  ::benchmark::Initialize(&argc, argv);
  ::benchmark::RunSpecifiedBenchmarks();
  return 0;
}
