// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/obstacles_critic.hpp"

namespace mppi::critics
{

void ObstaclesCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 2);
  getParam(weight_, "cost_weight", 1.25);
  getParam(collision_cost_, "collision_cost", 2000.0);

  collision_checker_.setCostmap(costmap_);
  possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
  RCLCPP_INFO(
    logger_,
    "ObstaclesCritic instantiated with %d power and %f weight. "
    "Critic will collision check based on %s cost.",
    power_, weight_, consider_footprint_ ? "footprint" : "circular");
}

unsigned char ObstaclesCritic::findCircumscribedCost(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  double result = -1.0;
  bool inflation_layer_found = false;
  // check if the costmap has an inflation layer
  for (auto layer = costmap->getLayeredCostmap()->getPlugins()->begin();
    layer != costmap->getLayeredCostmap()->getPlugins()->end();
    ++layer)
  {
    auto inflation_layer = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer);
    if (!inflation_layer) {
      continue;
    }

    inflation_layer_found = true;
    double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
    double resolution = costmap->getCostmap()->getResolution();
    result = inflation_layer->computeCost(circum_radius / resolution);
  }

  if (!inflation_layer_found) {
    RCLCPP_WARN(
      rclcpp::get_logger("computeCircumscribedCost"),
      "No inflation layer found in costmap configuration. "
      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
      "field to speed up collision checking by only checking the full footprint "
      "when robot is within possibly-inscribed radius of an obstacle. This may "
      "significantly slow down planning times!");
  }

  return result;
}

void ObstaclesCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  bool all_trajectories_collide = true;
  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
    bool trajectory_collide = false;
    unsigned char trajectory_cost = nav2_costmap_2d::FREE_SPACE;

    for (size_t j = 0; j < data.trajectories.x.shape(1); j++) {
      unsigned char pose_cost = costAtPose(
        data.trajectories.x(i, j), data.trajectories.y(i, j), data.trajectories.yaws(i, j));
      trajectory_cost = std::max(trajectory_cost, pose_cost);

      if (inCollision(trajectory_cost)) {
        trajectory_collide = true;
        break;
      }
    }

    if (!trajectory_collide) {all_trajectories_collide = false;}
    data.costs[i] +=
      trajectory_collide ? collision_cost_ : scoreCost(trajectory_cost);

  }

  data.fail_flag = all_trajectories_collide;
}

unsigned char ObstaclesCritic::costAtPose(double x, double y, double theta)
{
  unsigned char cost;
  unsigned int x_i, y_i;
  collision_checker_.worldToMap(x, y, x_i, y_i);
  cost = static_cast<unsigned char>(collision_checker_.pointCost(x_i, y_i));

  if (consider_footprint_ && cost >= possibly_inscribed_cost_) {
    cost = static_cast<unsigned char>(collision_checker_.footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint()));
  }

  return cost;
}

bool ObstaclesCritic::inCollision(unsigned char cost) const
{
  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  switch (cost) {
    using namespace nav2_costmap_2d; // NOLINT
    case (LETHAL_OBSTACLE):
      return true;
    case (INSCRIBED_INFLATED_OBSTACLE):
      return consider_footprint_ ? false : true;
    case (NO_INFORMATION):
      return is_tracking_unknown ? false : true;
  }

  return false;
}

unsigned char ObstaclesCritic::maxCost()
{
  return consider_footprint_ ? nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE :
         nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
}

double ObstaclesCritic::scoreCost(unsigned char cost_arg)
{
  double max_cost = static_cast<double>(maxCost());

  double cost = static_cast<double>(cost_arg) / max_cost;

  return pow(cost * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ObstaclesCritic,
  mppi::critics::CriticFunction)
