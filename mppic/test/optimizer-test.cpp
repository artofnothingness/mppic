#include <cstddef>
#include <nav2_costmap_2d/cost_values.hpp>
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


void printMap(nav2_costmap_2d::Costmap2D & costmap){
  printf("map:\n");
  for (unsigned int i = 0; i < costmap.getSizeInCellsY(); i++) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsX(); j++) {
      printf("%4d", static_cast<int>(costmap.getCost(j, i)));
    }
    printf("\n\n");
  }
}

void setUpOptimizerParams(std::vector<rclcpp::Parameter> &params_){
  params_.push_back(rclcpp::Parameter("TestNode.iteration_count", 300));
}

void addObstacle(nav2_costmap_2d::Costmap2D & costmap, unsigned int mx, 
  unsigned int my, unsigned int size, unsigned char cost){

  for (unsigned int i = mx; i < mx+size; i++) {
    for (unsigned int j = my; j < my+size; j++) {
      costmap.setCost(i, j, cost);
    }
  }
}

bool checkTrajectoryCollision(auto & trajectory, nav2_costmap_2d::Costmap2D & costmap){
  
  unsigned int point_mx, point_my;
  for (auto i = trajectory.begin(); i < trajectory.end(); i+=3){
    costmap.worldToMap(*i, *(i+1), point_mx, point_my);
    auto cost_ = costmap.getCost(point_mx, point_my);
    if (cost_ > nav2_costmap_2d::FREE_SPACE ||
        cost_ == nav2_costmap_2d::NO_INFORMATION){
      return true;    
    }
  }
  return false;
}


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



TEST_CASE("Optimizer with obstacles", "") {

  using T = float;

  std::string node_name = "TestNode";

  std::vector<rclcpp::Parameter> params_;
  rclcpp::NodeOptions options;
  setUpOptimizerParams(params_);
  options.parameter_overrides(params_);

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");
  auto st = rclcpp_lifecycle::State{};

  auto & model = mppi::models::NaiveModel<T>;

  auto optimizer = mppi::optimization::Optimizer<T>();

  costmap_ros->on_configure(st);
  *costmap_ros->getCostmap() = *std::make_shared<nav2_costmap_2d::Costmap2D>(30, 30, 0.1, 0, 0, 0);
  addObstacle(*costmap_ros->getCostmap(), 5, 5, 20, 255);
  // printMap(*costmap_ros->getCostmap());
  optimizer.on_configure(node, node_name, costmap_ros, model);
  optimizer.on_activate();

  size_t poses_count = GENERATE(10, 30, 100);

  SECTION("Running evalTrajectoryFromControlSequence") {
    
    std::string frame = "odom";
    auto time = node->get_clock()->now();

    nav_msgs::msg::Path path;                  // reference path
    geometry_msgs::msg::PoseStamped ps;        // goal point in reference path
    geometry_msgs::msg::Twist twist;           // initial robot velocity
    geometry_msgs::msg::PoseStamped init_cond; // initial robot pose

    // lambda expression for setting header
    auto setHeader = [&](auto &&msg) {
      msg.header.frame_id = frame;
      msg.header.stamp = time;
    };

    // lambda expression for refernce path generation
    auto fillRealPath = [&](size_t count) {
      for (size_t i = 0; i < count; i++) {
          ps.pose.position.x = i*0.02;
          ps.pose.position.y = i*0.02;
          path.poses.push_back(ps);
      }
    };  

    setHeader(ps);
    setHeader(init_cond);
    setHeader(path);

    fillRealPath(poses_count);

    // std::cout<< "REFERENCE PATH" << std::endl;
    // for (auto item:path.poses){
    //    std::cout<< "x = "<< item.pose.position.x <<" y = "<< item.pose.position.y << std::endl;
    // }

    CHECK_NOTHROW(optimizer.evalNextBestControl(init_cond, twist, path));
    CHECK_NOTHROW(optimizer.evalTrajectoryFromControlSequence(init_cond, twist));

    auto trajectory = optimizer.evalTrajectoryFromControlSequence(init_cond, twist);
    // std::cout<< "TRAJECTORY!" << std::endl;
    // std::cout << trajectory << std::endl;
    bool result = checkTrajectoryCollision(trajectory, *costmap_ros->getCostmap());
    REQUIRE(result == 0 );
  }

  optimizer.on_deactivate();
  optimizer.on_cleanup();
  costmap_ros->on_cleanup(st);
  costmap_ros.reset();

}
