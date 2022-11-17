// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include <cmath>
#include "mppic/critics/obstacles_critic.hpp"

namespace mppi::critics
{

void ObstaclesCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 2);
  getParam(weight_, "cost_weight", 2.0);
  getParam(collision_cost_, "collision_cost", 2000.0);
  getParam(trajectory_penalty_distance_, "trajectory_penalty_distance", 1.0);
  getParam(collision_margin_distance_, "collision_margin_distance", 0.12);
  getParam(near_goal_distance_, "near_goal_distance", 0.5);

  collision_checker_.setCostmap(costmap_);
  possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
  RCLCPP_INFO(
    logger_,
    "ObstaclesCritic instantiated with %d power and %f weight. "
    "Critic will collision check based on %s cost.",
    power_, weight_, consider_footprint_ ? "footprint" : "circular");
}

double ObstaclesCritic::findCircumscribedCost(
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
    const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
    const double resolution = costmap->getCostmap()->getResolution();
    result = inflation_layer->computeCost(circum_radius / resolution);
    inflation_scale_factor_ = static_cast<float>(inflation_layer->getCostScalingFactor());
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

float ObstaclesCritic::distanceToObstacle(const CollisionCost & cost)
{
  const float scale_factor = inflation_scale_factor_;
  const float min_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  float dist_to_obj = (scale_factor * min_radius - log(cost.cost) + log(253.0f)) / scale_factor;

  // If not footprint collision checking, the cost is using the center point cost and
  // needs the radius subtracted to obtain the closest distance to the object
  if (!cost.using_footprint) {
    dist_to_obj -= min_radius;
  }

  return dist_to_obj;
}

void ObstaclesCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_) {
    return;
  }

  // If near the goal, don't apply the preferential term since the goal is near obstacles
  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
    near_goal = true;
  }

  auto && raw_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  const size_t traj_len = data.trajectories.x.shape(1);
  bool all_trajectories_collide = true;
  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
    bool trajectory_collide = false;
    float traj_cost = 0.0;
    const auto & traj = data.trajectories;
    float repulsion_cost = 0.0;

    for (size_t j = 0; j < traj_len; j++) {
      const CollisionCost pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), traj.yaws(i, j));

      if (inCollision(pose_cost.cost)) {
        trajectory_collide = true;
        break;
      }

      const float dist_to_obj = distanceToObstacle(pose_cost);
      if (dist_to_obj < collision_margin_distance_) {
        // Near-collision, all points must be punished
        traj_cost += (collision_margin_distance_ - dist_to_obj);
      } else if (dist_to_obj < trajectory_penalty_distance_ && !near_goal) {
        // Prefer general trajectories further from obstacles
        repulsion_cost = std::max(repulsion_cost, (trajectory_penalty_distance_ - dist_to_obj));
      }
    }

    traj_cost += repulsion_cost;
    if (!trajectory_collide) {all_trajectories_collide = false;}
    raw_cost[i] = static_cast<float>(trajectory_collide ? collision_cost_ : traj_cost);
  }

  data.costs = xt::pow(raw_cost * weight_, power_);
  data.fail_flag = all_trajectories_collide;
}

CollisionCost ObstaclesCritic::costAtPose(float x, float y, float theta)
{
  CollisionCost collision_cost;
  float & cost = collision_cost.cost;
  collision_cost.using_footprint = false;
  unsigned int x_i, y_i;
  collision_checker_.worldToMap(x, y, x_i, y_i);
  cost = collision_checker_.pointCost(x_i, y_i);

  if (consider_footprint_ && cost >= possibly_inscribed_cost_) {
    cost = static_cast<float>(collision_checker_.footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint()));
    collision_cost.using_footprint = true;
  }

  return collision_cost;
}

bool ObstaclesCritic::inCollision(float cost) const
{
  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  switch (static_cast<unsigned char>(cost)) {
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

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ObstaclesCritic,
  mppi::critics::CriticFunction)
