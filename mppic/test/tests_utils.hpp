
#pragma once

#include <iostream>
#include <algorithm>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include <rclcpp/executors.hpp>


/**
 * Print costmap to stdout.
 * @param costmap map to be printed.
*/
void printMap(const nav2_costmap_2d::Costmap2D & costmap){
  for (unsigned int i = 0; i < costmap.getSizeInCellsY(); i++) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsX(); j++) {
      printf("%4d", static_cast<int>(costmap.getCost(j, i)));
    }
    printf("\n\n");
  }
}

/**
 * Print costmap with trajectory and goal point to stdout.
 * @param costmap map to be printed.
 * @param trajectory trajectory container (xt::tensor) to be printed.
 * @param goal_point goal point to be printed.
*/
void printMapWithGoalAndTrajectory(nav2_costmap_2d::Costmap2D & costmap, const auto & trajectory, 
    const geometry_msgs::msg::PoseStamped & goal_point){
  std::cout<<"map with trajectory: \ntrajectory point = 1 \ngoal point = 100 \nobsctacle = 255\n"<<std::endl;

  // create new costmap
  nav2_costmap_2d::Costmap2D costmap2d(
    costmap.getSizeInCellsX(),
    costmap.getSizeInCellsY(),
    costmap.getResolution(),
    costmap.getOriginX(),
    costmap.getOriginY(),
    costmap.getDefaultValue()
  );

  // copy obstacles from original costmap
  costmap2d = costmap;
  unsigned int point_mx = 0;
  unsigned int point_my = 0;
  unsigned char trajectory_point = 1;
  unsigned char goal_point_cost = 100;

  // add trajectory on map
  for (size_t i = 0; i < trajectory.shape()[0]; ++i){
    costmap2d.worldToMap(trajectory(i, 0), trajectory(i, 1), point_mx, point_my);
    costmap2d.setCost(point_mx, point_my, trajectory_point);
  }
  // add goal point on map
    costmap2d.worldToMap(
        goal_point.pose.position.x,
        goal_point.pose.position.y,
        point_mx, point_my);
    
    costmap2d.setCost(point_mx, point_my, goal_point_cost);

    printMap(costmap2d);

}

/**
 * Add a square obstacle to the costmap.
 * @param costmap map to be modified.
 * @param upper_left_corner_x obstacle upper left corner X coord (on the costmap).
 * @param upper_left_corner_y obstacle upper left corner Y coord (on the costmap).
 * @param size obstacle side size.
 * @param cost obstacle value on costmap.
*/
void addObstacle(nav2_costmap_2d::Costmap2D & costmap, unsigned int upper_left_corner_x, 
  unsigned int upper_left_corner_y, unsigned int size, unsigned char cost){
  for (unsigned int i = upper_left_corner_x; i < upper_left_corner_x+size; i++) {
    for (unsigned int j = upper_left_corner_y; j < upper_left_corner_y+size; j++) {
      costmap.setCost(i, j, cost);
    }
  }
}

/**
 * Check the trajectory for collisions with obstacles on the map.
 * @param trajectory trajectory container (xt::tensor) to be checked.
 * @param costmap costmap with obstacles
 * @return true - if the trajectory crosses an obstacle on the map, false - if not
*/
bool inCollision(const auto & trajectory, const nav2_costmap_2d::Costmap2D & costmap){
  unsigned int point_mx = 0;
  unsigned int point_my = 0; 

  for (size_t i = 0; i < trajectory.shape()[0]; ++i){
    costmap.worldToMap(trajectory(i, 0), trajectory(i, 1), point_mx, point_my);
    auto cost_ = costmap.getCost(point_mx, point_my);
    if (cost_ > nav2_costmap_2d::FREE_SPACE ||
        cost_ == nav2_costmap_2d::NO_INFORMATION){
      return true;    
    }
  }
  return false;
}
