// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/obstacles_critic.hpp"

#include "execution"
#include "atomic"

namespace mppi::critics
{

void ObstaclesCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(consider_footprint_, "consider_footprint", true);
  getParam(power_, "cost_power", 2);
  getParam(weight_, "cost_weight", 1.25);
  getParam(collision_cost_, "collision_cost", 2000.0);

  collision_checker_.setCostmap(costmap_);
  RCLCPP_INFO(
    logger_,
    "ObstaclesCritic instantiated with %d power and %f weight. "
    "Critic will collision check based on %s cost.",
    power_, weight_, consider_footprint_ ? "footprint" : "circular");
}

void ObstaclesCritic::score(models::CriticFunctionData & data)
{
  if (!enabled_) {
    return;
  }

  std::atomic<bool> all_trajectories_collide = true;

  std::vector<size_t> trajectories_idxs(data.trajectories.shape(0));
  std::iota(trajectories_idxs.begin(), trajectories_idxs.end(), 0);

  std::for_each(
    std::execution::par_unseq, trajectories_idxs.begin(), trajectories_idxs.end(),
    [&](int i) {
      bool trajectory_collide = false;
      unsigned char trajectory_cost = nav2_costmap_2d::FREE_SPACE;

      for (size_t j = 0; j < data.trajectories.shape(1); j++) {
        unsigned char pose_cost = costAtPose(
          data.trajectories(i, j, 0), data.trajectories(i, j, 1), data.trajectories(i, j, 2));
        trajectory_cost = std::max(trajectory_cost, pose_cost);

        if (inCollision(trajectory_cost)) {
          trajectory_collide = true;
          break;
        }
      }

      if (!trajectory_collide) {all_trajectories_collide.store(false);}
      data.costs[i] +=
      trajectory_collide ? collision_cost_ : scoreCost(trajectory_cost);

    });

  data.fail_flag = all_trajectories_collide;
}

unsigned char ObstaclesCritic::costAtPose(double x, double y, double theta)
{
  unsigned char cost;
  if (consider_footprint_) {
    cost = static_cast<unsigned char>(collision_checker_.footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint()));
  } else {
    unsigned int x_i, y_i;
    collision_checker_.worldToMap(x, y, x_i, y_i);
    cost = static_cast<unsigned char>(collision_checker_.pointCost(x_i, y_i));
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
    default:
      return false;
  }
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
