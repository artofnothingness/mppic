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



/**
 * Adds some parameters for the optimizer to a special container.
 *
 * @param params_ container for optimizer's parameters.
*/
void setUpOptimizerParams(std::vector<rclcpp::Parameter> &params_){
  params_.push_back(rclcpp::Parameter("TestNode.iteration_count", 5));
  params_.push_back(rclcpp::Parameter("TestNode.lookahead_dist", 5));
  params_.push_back(rclcpp::Parameter("TestNode.time_steps", 30));
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


TEST_CASE("Optimizer with costmap2d and obstacles", "[collision]") {

  using T = float;

  std::string node_name = "TestNode";
  auto state = rclcpp_lifecycle::State{};
  std::vector<rclcpp::Parameter> params_;
  rclcpp::NodeOptions options;
  setUpOptimizerParams(params_);
  options.parameter_overrides(params_);

  // args for costmap2d
  unsigned int cells_size_x = 20;
  unsigned int cells_size_y = 20;
  double resolution = 0.1;
  double origin_x = 0.0;
  double origin_y = 0.0;
  unsigned char default_value = 0;
  
  // args for obstacle on costmap2d
  const unsigned int upper_left_corner_x = 5;  
  const unsigned int upper_left_corner_y = 7;  
  const unsigned int obstacle_side_size_cells = 9;  
  unsigned char obstacle_cost = 255;                                     
  
  // create parameters for reference path generation
  size_t poses_count = 40;
  float x_step = 0.015;
  float y_step = 0.035;

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");
  
  auto & model = mppi::models::NaiveModel<T>;
  auto optimizer = mppi::optimization::Optimizer<T>();

  costmap_ros->on_configure(state);
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
              upper_left_corner_y, obstacle_side_size_cells, obstacle_cost);

  optimizer.on_configure(node, node_name, costmap_ros, model);
  optimizer.on_activate();

  float start_point_x = GENERATE(1.0, 0.4);
  float start_point_y = 0.4;

  SECTION("evalNextBestControl doesn't produce path crossing the obstacles") {
    
    std::string frame = "odom";              // frame for header in path and points
    auto time = node->get_clock()->now();    // time for header in path

    nav_msgs::msg::Path reference_path;
    geometry_msgs::msg::PoseStamped reference_goal_pose;
    geometry_msgs::msg::PoseStamped init_robot_pose;   
    init_robot_pose.pose.position.x = start_point_x;
    init_robot_pose.pose.position.y = start_point_y;          
    geometry_msgs::msg::Twist init_robot_vel;           

    // lambda expression for setting header
    auto setHeader = [&](auto &&msg) {
      msg.header.frame_id = frame;
      msg.header.stamp = time;
    };

    // lambda expression for refernce path generation
    auto fillRealPath = [&](size_t count) {
      for (size_t i = 0; i < count; i++) {
          reference_goal_pose.pose.position.x = i*x_step + start_point_x;
          reference_goal_pose.pose.position.y = i*y_step + start_point_y;
          reference_path.poses.push_back(reference_goal_pose);
      }
    };  

    setHeader(reference_goal_pose);
    setHeader(init_robot_pose);
    setHeader(reference_path);

    fillRealPath(poses_count);

    // update controal sequence in optimizer
    CHECK_NOTHROW(optimizer.evalNextBestControl(init_robot_pose, init_robot_vel, reference_path));
    // get best trajectory from optimizer
    auto trajectory = optimizer.evalTrajectoryFromControlSequence(init_robot_pose, init_robot_vel);
#ifdef TEST_DEBUG_INFO
    printMapWithGoalAndTrajectory(*costmap_ros->getCostmap(), trajectory, reference_goal_pose);
#endif   
    CHECK(!inCollision(trajectory, *costmap_ros->getCostmap()));
  }

  optimizer.on_deactivate();
  optimizer.on_cleanup();
  costmap_ros->on_cleanup(state);
  costmap_ros.reset();

}
