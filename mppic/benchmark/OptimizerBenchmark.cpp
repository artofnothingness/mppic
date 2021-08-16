#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "mppi/Models.hpp"
#include "mppi/impl/Optimizer.hpp"
#include <benchmark/benchmark.h>

template <typename T>
void setDefaultHeader(T &msg) {
  msg.header.frame_id = "map";
  msg.header.stamp.nanosec = 0;
  msg.header.stamp.sec = 0;
}

geometry_msgs::msg::PoseStamped createPose() {
  geometry_msgs::msg::PoseStamped pose;
  setDefaultHeader(pose);
  pose.pose.position.x = 1;
  pose.pose.position.y = 1;
  pose.pose.position.z = 1;
  return pose;
}

geometry_msgs::msg::Twist createTwist() {
  geometry_msgs::msg::Twist twist;

  twist.linear.x = 1;
  twist.linear.y = 0;
  twist.linear.z = 0;
  return twist;
}

nav_msgs::msg::Path createPath() {
  nav_msgs::msg::Path path;
  setDefaultHeader(path);

  for (int i = 0; i < 100; i++) {
    geometry_msgs::msg::PoseStamped p;
    setDefaultHeader(p);
    p.pose.position.x = i;
    p.pose.position.y = i * i;
    p.pose.position.z = 0;
    path.poses.push_back(p);
  }

  return path;
}

class OptimizerBenchmark : public benchmark::Fixture {
  using T = float;

public:
  void SetUp(const ::benchmark::State &state) {
    (void)state;

    std::string node_name = "BenchmarkNode";
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name);
    costmap_ = new nav2_costmap_2d::Costmap2D(500, 500, 0.1, 0, 0, 100);
    auto &model = mppi::models::NaiveModel<float>;

    optimizer_ = mppi::optimization::Optimizer<T>(node_, node_name, costmap_, model);
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
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  mppi::optimization::Optimizer<T> optimizer_;
  nav2_costmap_2d::Costmap2D *costmap_;
};

BENCHMARK_DEFINE_F(OptimizerBenchmark, evalNextControlBenchmark)
(benchmark::State &st) {
  for (auto _ : st) {
    geometry_msgs::msg::Twist twist = createTwist();
    nav_msgs::msg::Path path = createPath();
    auto result = optimizer_.evalNextControl(twist, path);
  }
}

BENCHMARK_REGISTER_F(OptimizerBenchmark, evalNextControlBenchmark)->Unit(benchmark::kMillisecond);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  ::benchmark::Initialize(&argc, argv);
  ::benchmark::RunSpecifiedBenchmarks();
  return 0;
}
