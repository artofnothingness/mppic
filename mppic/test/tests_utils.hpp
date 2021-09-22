
#include <iostream>
#include <algorithm>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include <rclcpp/executors.hpp>

void printMap(const nav2_costmap_2d::Costmap2D & costmap){
  for (unsigned int i = 0; i < costmap.getSizeInCellsY(); i++) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsX(); j++) {
      printf("%4d", static_cast<int>(costmap.getCost(j, i)));
    }
    printf("\n\n");
  }
}


void printMapWthGoalAndTrajectory(nav2_costmap_2d::Costmap2D & costmap, const auto & trajectory, 
    const geometry_msgs::msg::PoseStamped & goal_point){
  printf("map with trajectory: \ntrajectory point = 1 \ngoal point = 2 \nobsctacle = 255\n");

  // create new costmap
  nav2_costmap_2d::Costmap2D costmap2d(
      costmap.getSizeInCellsX(),
      costmap.getSizeInCellsY(),
      costmap.getResolution(),
      costmap.getOriginX(),
      costmap.getOriginY(),
      costmap.getDefaultValue());

  // copy obstacles from original costmap
  costmap2d = costmap;
  unsigned int point_mx, point_my;
  unsigned char trajectory_point = 1;

  // add trajectory on map
  for (auto i = trajectory.begin(); i < trajectory.end(); i+=3){
    costmap2d.worldToMap(*i, *(i+1), point_mx, point_my);
    costmap2d.setCost(point_mx, point_my, trajectory_point);
  }
  // add goal point on map
    costmap2d.worldToMap(
        goal_point.pose.position.x,
        goal_point.pose.position.y,
        point_mx, point_my);
    
    costmap2d.setCost(point_mx, point_my, 2);

    printMap(costmap2d);

}


void setUpOptimizerParams(std::vector<rclcpp::Parameter> &params_){
  params_.push_back(rclcpp::Parameter("TestNode.iteration_count", 300));
  params_.push_back(rclcpp::Parameter("TestNode.lookahead_dist", 5));
  params_.push_back(rclcpp::Parameter("TestNode.time_steps", 60));
}

void addObstacle(nav2_costmap_2d::Costmap2D & costmap, const unsigned int & mx, 
  const unsigned int & my, const unsigned int & size, const unsigned char & cost){

  for (unsigned int i = mx; i < mx+size; i++) {
    for (unsigned int j = my; j < my+size; j++) {
      costmap.setCost(i, j, cost);
    }
  }
}

bool checkTrajectoryCollision(const auto & trajectory, const nav2_costmap_2d::Costmap2D & costmap){
  
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
