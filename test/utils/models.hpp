// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#pragma once
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>

struct TestOptimizerSettings
{
  int batch_size;
  int time_steps;
  int iteration_count;
  double lookahead_distance;
  std::string motion_model;
  bool consider_footprint;
};

struct TestPose
{
  double x;
  double y;
};

struct TestCostmapSettings
{
  const unsigned int cells_x = 40;
  const unsigned int cells_y = 40;
  const double origin_x = 0.0;
  const double origin_y = 0.0;
  const double resolution = 0.1;
  const unsigned char cost_map_default_value = 0;
  const double footprint_size = 0.15;

  std::pair<unsigned int, unsigned int> getCenterIJ()
  {
    return {
      cells_x / 2,
      cells_y / 2};
  }

  TestPose getCenterPose()
  {
    return {
      static_cast<double>(cells_x) * resolution / 2.0,
      static_cast<double>(cells_y) * resolution / 2.0};
  }
};
struct TestObstaclesSettings
{
  unsigned int center_cells_x;
  unsigned int center_cells_y;
  unsigned int obstacle_size;
  unsigned char obstacle_cost;
};

struct TestPathSettings
{
  TestPose start_pose;
  unsigned int poses_count;
  double step_x;
  double step_y;
};
