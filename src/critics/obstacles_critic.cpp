#include "mppic/critics/obstacles_critic.hpp"

namespace mppi::critics
{

void ObstaclesCritic::initialize()
{
  auto node = parent_.lock();
  auto getParam = utils::getParamGetter(node, name_);
  getParam(consider_footprint_, "consider_footprint", true);
  getParam(power_, "obstacle_cost_power", 1);
  getParam(weight_, "obstacle_cost_weight", 20);
  getParam(inflation_cost_scaling_factor_, "inflation_cost_scaling_factor", 3.0);
  getParam(inflation_radius_, "inflation_radius", 0.75);

  inscribed_radius_ = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  collision_checker_.setCostmap(costmap_);
  RCLCPP_INFO(
    logger_,
    "ObstaclesCritic instantiated with %d power and %f weight. "
    "It is using %f inflation scale and %f inflation radius to compute distances."
    "Critic will collision check based on %s cost.",
    power_, weight_, inflation_cost_scaling_factor_,
    inflation_radius_, consider_footprint_ ? "footprint" : "circular");
}

void ObstaclesCritic::score(
  const geometry_msgs::msg::PoseStamped & /*robot_pose*/, const xt::xtensor<double, 3> & trajectories,
  const xt::xtensor<double, 2> & /*path*/, xt::xtensor<double, 1> & costs)
{
  constexpr double collision_cost_value = std::numeric_limits<double>::max() / 2;


  enum TrajectoryState : uint8_t { Collision, Inflated, Free };

  for (size_t i = 0; i < trajectories.shape()[0]; ++i) {
    double min_dist_to_obstacle = std::numeric_limits<double>::max();
    TrajectoryState state = Free;

    for (size_t j = 0; j < trajectories.shape()[1]; ++j) {
      auto point = xt::view(trajectories, i, j, xt::all());
      unsigned char cost = costAtPose(point);
      if (!isFree(cost)) {
        if (inCollision(cost)) {
          state = Collision;
          break;
        }
        state = Inflated;
        min_dist_to_obstacle = std::min(toDist(cost), min_dist_to_obstacle);
      }
    }

    if (state == Inflated) {
      costs[i] += scoreDistance(min_dist_to_obstacle);
    } else if (state == Collision) {
      costs[i] += collision_cost_value;
    }
  }
}

unsigned char ObstaclesCritic::costAtPose(const auto & point)
{
  unsigned char cost;

  if (consider_footprint_) {
    cost = static_cast<unsigned char>(collision_checker_.footprintCostAtPose(
      point(0), point(1), point(2), costmap_ros_->getRobotFootprint()));
  } else {
    cost = static_cast<unsigned char>(collision_checker_.pointCost(point(0), point(1)));
  }

  return cost;
}

bool ObstaclesCritic::inCollision(unsigned char & cost) const
{
  unsigned char max_valid_cost;
  if (consider_footprint_) {
    max_valid_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
  } else {
    max_valid_cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
    return cost >= max_valid_cost && cost != nav2_costmap_2d::NO_INFORMATION;
  }

  return cost >= max_valid_cost;
}

bool ObstaclesCritic::isFree(unsigned char & cost) const
{
	return cost == nav2_costmap_2d::FREE_SPACE;
}

double ObstaclesCritic::toDist(unsigned char & cost)
{
  return (-1.0 / inflation_cost_scaling_factor_) *
           std::log(
             static_cast<double>(cost) /
             (static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) - 1.0)) +
         inscribed_radius_;
}

double ObstaclesCritic::scoreDistance(double & min_dist)
{
  return static_cast<double>(pow((1.01 * inflation_radius_ - min_dist) * weight_, power_));
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ObstaclesCritic, mppi::critics::CriticFunction)
