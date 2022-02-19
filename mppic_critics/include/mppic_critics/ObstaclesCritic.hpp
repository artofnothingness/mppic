#include <xtensor/xtensor.hpp>

#include "mppic_base/CriticFunction.hpp"

#include "mppic/utils/LineIterator.hpp"
#include "mppic/utils/common.hpp"
#include "mppic/utils/geometry.hpp"

namespace mppi::optimization {

template <typename T>
class ObstaclesCritic : public CriticFunction<T>
{
public:
  void getParams() final
  {
    auto getParam = utils::getParamGetter(this->parent_, this->node_name_);
    getParam(power_, "obstacle_cost_power", 2);
    getParam(weight_, "obstacle_cost_weight", 10);
    getParam(inflation_cost_scaling_factor_, "inflation_cost_scaling_factor", 3.0);
    getParam(inflation_radius_, "inflation_radius", 0.75);

    inscribed_radius_ = this->costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  }

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  virtual void score(
    const geometry_msgs::msg::PoseStamped & robot_pose, const xt::xtensor<T, 3> & trajectories,
    const xt::xtensor<T, 2> & path, xt::xtensor<T, 1> & costs) final
  {
    (void)robot_pose;
    (void)path;

    constexpr T collision_cost_value = std::numeric_limits<T>::max() / 2;

    auto dist_by_cost = [this](const auto cost) {
      return (-1.0 / inflation_cost_scaling_factor_) *
               std::log(
                 static_cast<double>(cost) /
                 (static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) - 1.0)) +
             inscribed_radius_;
    };

    for (size_t i = 0; i < trajectories.shape()[0]; ++i) {
      double min_dist = std::numeric_limits<double>::max();
      bool inflated = false;
      for (size_t j = 0; j < trajectories.shape()[1]; ++j) {
        auto footprint = getOrientedFootprint(
          xt::view(trajectories, i, j), this->costmap_ros_->getRobotFootprint());
        auto cost = static_cast<unsigned char>(scoreFootprint(footprint));

        if (inCollision(cost)) {
          costs[i] = collision_cost_value;
          inflated = false;
          break;
        }

        if (cost != nav2_costmap_2d::FREE_SPACE) {
          min_dist = std::min(dist_by_cost(cost), min_dist);
          inflated = true;
        }
      }

      if (inflated) {
        costs[i] += static_cast<T>(pow((1.01 * inflation_radius_ - min_dist) * weight_, power_));
      }
    }
  }

private:
  bool inCollision(unsigned char cost) const
  {
    if (this->costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
      return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
             cost != nav2_costmap_2d::NO_INFORMATION;
    }

    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  std::vector<geometry_msgs::msg::Point> getOrientedFootprint(
    const auto & robot_pose, const std::vector<geometry_msgs::msg::Point> & footprint_spec) const
  {
    std::vector<geometry_msgs::msg::Point> oriented_footprint;
    oriented_footprint.resize(footprint_spec.size());

    double cos_yaw = cos(static_cast<double>(robot_pose(2)));
    double sin_yaw = sin(static_cast<double>(robot_pose(2)));

    for (size_t i = 0; i < footprint_spec.size(); ++i) {
      oriented_footprint[i].x = static_cast<double>(robot_pose(0)) + footprint_spec[i].x * cos_yaw -
                                footprint_spec[i].y * sin_yaw;
      oriented_footprint[i].y = static_cast<double>(robot_pose(1)) + footprint_spec[i].x * sin_yaw +
                                footprint_spec[i].y * cos_yaw;
    }

    return oriented_footprint;
  }

  double lineCost(int x0, int x1, int y0, int y1) const
  {
    double line_cost = 0.0;
    double point_cost = -1.0;

    for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
      point_cost = static_cast<double>(this->costmap_->getCost(
        static_cast<unsigned int>(line.getX()), static_cast<unsigned int>(line.getY())));

      if (line_cost < point_cost) {
        line_cost = point_cost;
      }
    }

    return line_cost;
  }

  double scoreFootprint(const std::vector<geometry_msgs::msg::Point> & footprint) const
  {
    unsigned int x0{0};
    unsigned int x1{0};
    unsigned int y0{0};
    unsigned int y1{0};

    double line_cost = 0.0;
    double footprint_cost = 0.0;

    auto world_to_map = [&](size_t i, unsigned int & x, unsigned int & y) {
      if (!this->costmap_->worldToMap(footprint[i].x, footprint[i].y, x, y)) {
        throw std::runtime_error("Footprint Goes Off Grid.");
      }
    };

    for (unsigned int i = 0; i < footprint.size(); ++i) {
      world_to_map(i, x0, y0);
      world_to_map(i != footprint.size() - 1 ? i + 1 : 0, x1, y1);

      line_cost = lineCost(
        static_cast<int>(x0), static_cast<int>(x1), static_cast<int>(y0), static_cast<int>(y1));

      footprint_cost = std::max(line_cost, footprint_cost);
    }

    return footprint_cost;
  }

  double inflation_cost_scaling_factor_{0};
  double inscribed_radius_{0};
  double inflation_radius_{0};
  unsigned int power_{0};
  double weight_{0};
};

} // namespace mppi::optimization
