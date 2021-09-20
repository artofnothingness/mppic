#define CATCH_CONFIG_RUNNER

#ifdef DO_BENCHMARKS
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#endif

#include <catch2/catch.hpp>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include <rclcpp/executors.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "mppi/Models.hpp"
#include "mppi/impl/Optimizer.hpp"

#include "xtensor/xarray.hpp"
#include <xtensor/xview.hpp>
#include <xtensor/xio.hpp>
#include <iostream>


void printMap(nav2_costmap_2d::Costmap2D & costmap)
{
  printf("map:\n");
  for (unsigned int i = 0; i < costmap.getSizeInCellsY(); i++) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsX(); j++) {
      printf("%4d", static_cast<int>(costmap.getCost(j, i)));
    }
    printf("\n\n");
  }
}


TEST_CASE("Optimizer evaluates Next Control", "") {
  using T = float;

  std::string node_name = "TestNode";
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name);

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");
  auto st =  rclcpp_lifecycle::State{};

  auto &model = mppi::models::NaiveModel<T>;

  auto optimizer = mppi::optimization::Optimizer<T>(node, node_name, costmap_ros, model);

  costmap_ros->on_configure(st);
  optimizer.on_configure();
  optimizer.on_activate();

  size_t poses_count = GENERATE(10, 30, 100);

  SECTION("Running evalNextControl") {
    geometry_msgs::msg::Twist twist;

    std::string frame = "odom";
    auto time = node->get_clock()->now();

    auto setHeader = [&](auto &&msg) {
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


TEST_CASE("Optimizer with obstacles", "") {
  using T = float;

  std::string node_name = "TestNode";
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name);

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");
  auto st =  rclcpp_lifecycle::State{};

  auto &model = mppi::models::NaiveModel<T>;

  auto optimizer = mppi::optimization::Optimizer<T>(node, node_name, costmap_ros, model);

  costmap_ros->on_configure(st);

  *costmap_ros->getCostmap() = *std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 0.1, 0, 0, 0);
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros->getCostmap();  

  // loop through the costmap
  for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
      std::cout<< "i =" << i << " j = " << j <<std::endl;
      //compareCellToNeighbors(*costmap, i, j);
    }
  }

  optimizer.on_configure();
  optimizer.on_activate();
  
  size_t poses_count = GENERATE(10, 30, 100);

  SECTION("Running evalNextControl") {
    geometry_msgs::msg::Twist twist;

    std::string frame = "odom";
    auto time = node->get_clock()->now();

    auto setHeader = [&](auto &&msg) {
      msg.header.frame_id = frame;
      msg.header.stamp = time;
    };

    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped ps;
    setHeader(ps);
    setHeader(path);

    auto fillRealPath = [&](size_t count) {
      for (size_t i = 0; i < count; i++) {
          ps.pose.position.x = i*0.1;
          ps.pose.position.y = i*0.1;
          path.poses.push_back(ps);
      }
    };  

    fillRealPath(poses_count);

    auto trajectory = optimizer.evalTrajectoryFromControlSequence(ps);
    std::cout<< "PATH" << std::endl;
    for (auto item:path.poses){
       std::cout<< "x = "<< item.pose.position.x <<" y = "<< item.pose.position.y << std::endl;
    }

    std::cout<< "TRAJECTORY!" << std::endl;
    std::cout << trajectory << std::endl;

  CHECK_NOTHROW(optimizer.evalNextBestControl(ps, twist, path));
  CHECK_NOTHROW(optimizer.evalTrajectoryFromControlSequence(ps));

  }

  optimizer.on_deactivate();
  optimizer.on_cleanup();
  costmap_ros->on_cleanup(st);
  costmap_ros.reset();

}



int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  int result = Catch::Session().run(argc, argv);

  rclcpp::shutdown();
  return result;
}
