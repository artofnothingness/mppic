#define CATCH_CONFIG_RUNNER

#ifdef DO_BENCHMARKS
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#endif

#include <catch2/catch.hpp>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <rclcpp/executors.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "mppi/Models.hpp"
#include "mppi/impl/Optimizer.hpp"

TEST_CASE("Optimizer evaluates Next Control", "") {
  using T = float;

  std::string node_name = "TestNode";
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name);

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");
  auto st = rclcpp_lifecycle::State{};

  auto & model = mppi::models::NaiveModel<T>;

  auto optimizer =
    mppi::optimization::Optimizer<T>();

  costmap_ros->on_configure(st);
  optimizer.on_configure(node, node_name, costmap_ros, model);
  optimizer.on_activate();

  size_t poses_count = GENERATE(10, 30, 100);

  SECTION("Running evalNextControl") {
    geometry_msgs::msg::Twist twist;

    std::string frame = "odom";
    auto time = node->get_clock()->now();

    auto setHeader = [&](auto && msg) {
        msg.header.frame_id = frame;
        msg.header.stamp = time;
      };

    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped ps;
    setHeader(ps);
    setHeader(path);

    auto fillPath = [&](size_t count) {
        for (size_t i = 0; i < count; i++) {
          path.poses.push_back(ps);
        }
      };

    fillPath(poses_count);

    CHECK_NOTHROW(optimizer.evalNextBestControl(ps, twist, path));

#ifdef DO_BENCHMARKS
    WARN("Path with " << poses_count);
    BENCHMARK("evalNextControl Benchmark") {
      return optimizer.evalNextBestControl(ps, twist, path);
    };
#endif
  }

  optimizer.on_deactivate();
  optimizer.on_cleanup();
  costmap_ros->on_cleanup(st);
  costmap_ros.reset();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int result = Catch::Session().run(argc, argv);

  rclcpp::shutdown();
  return result;
}
