#ifdef DO_BENCHMARKS
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#endif

#include <catch2/catch.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
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
#include "tests_utils.hpp"


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



TEST_CASE("Optimizer with costmap2d and obstacles", "[collision]") {

  using T = float;

  std::string node_name = "TestNode";
  auto st = rclcpp_lifecycle::State{};
  std::vector<rclcpp::Parameter> params_;
  rclcpp::NodeOptions options;
  setUpOptimizerParams(params_);
  options.parameter_overrides(params_);

  // args for costmap2d
  unsigned int cells_size_x = 30;
  unsigned int cells_size_y = 30;
  double resolution = 0.1;
  double origin_x = 0.0;
  double origin_y = 0.0;
  unsigned char default_value = 0;
  
  // args for obstacle on costmap2d
  const unsigned int upper_left_corner_x = 5;  // x coord of the upper left corner of the obstacle on costmap
  const unsigned int upper_left_corner_y = 5;  // upper left corner of the obstacle on costmap
  const unsigned int obstacle_side_size = 18;  // in cells
  unsigned char cost = 255;                    // value of the obstacle on costmap                  
  
  // create parameters for reference path generation
  size_t poses_count = 100; // it can be GENERATE(100)
  float x_step = 0.025;
  float y_step = 0.01;

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");
  
  auto & model = mppi::models::NaiveModel<T>;
  auto optimizer = mppi::optimization::Optimizer<T>();

  costmap_ros->on_configure(st);
  *costmap_ros->getCostmap() = *std::make_shared<nav2_costmap_2d::Costmap2D>(
    cells_size_x, 
    cells_size_y,
    resolution,
    origin_x,
    origin_y,
    default_value
  );

  // creating a square obstacle
  addObstacle(*costmap_ros->getCostmap(), upper_left_corner_x, 
              upper_left_corner_y, obstacle_side_size, cost);

  optimizer.on_configure(node, node_name, costmap_ros, model);
  optimizer.on_activate();

  SECTION("evalNextBestControl doesn't produce path crossing the obstacles") {
    
    std::string frame = "odom";                // frame for header in path
    auto time = node->get_clock()->now();      // time for header in path

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
          ps.pose.position.x = i*x_step;
          ps.pose.position.y = i*y_step;
          path.poses.push_back(ps);
      }
    };  

    setHeader(ps);
    setHeader(init_cond);
    setHeader(path);

    fillRealPath(poses_count);

    // update controal sequence in optimizer
    CHECK_NOTHROW(optimizer.evalNextBestControl(init_cond, twist, path));
    // get best trajectory from optimizer
    auto trajectory = optimizer.evalTrajectoryFromControlSequence(init_cond, twist);
    // check trajectory for collision
    bool result = checkTrajectoryCollision(trajectory, *costmap_ros->getCostmap());
#ifdef TEST_DEBUG_INFO
    printMapWithGoalAndTrajectory(*costmap_ros->getCostmap(), trajectory, ps);
#endif
    REQUIRE(result == 0 );
  }

  optimizer.on_deactivate();
  optimizer.on_cleanup();
  costmap_ros->on_cleanup(st);
  costmap_ros.reset();

}
