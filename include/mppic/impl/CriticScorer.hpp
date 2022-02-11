#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xview.hpp>

#include "mppic/utils/LineIterator.hpp"
#include "mppic/utils/geometry.hpp"

namespace mppi::optimization {

// TODO configure from parent node
// TODO pluginize
template <typename T>
class CriticFunction {
public:
  CriticFunction() = default;
  virtual ~CriticFunction() = default;

  virtual void on_configure() = 0;
  virtual void score(const geometry_msgs::msg::PoseStamped &robot_pose,
                     const xt::xtensor<T, 3> &trajectories,
                     const xt::xtensor<T, 2> &path,
                     xt::xtensor<T, 1> &costs) = 0;
};

template <typename T>
class CriticScorer {
public:
  CriticScorer() = default;
  CriticScorer(std::vector<std::unique_ptr<CriticFunction<T>>> &&critics) {
    critics_ = std::move(critics);
  }

  std::vector<std::unique_ptr<CriticFunction<T>>> critics_;

  /**
   * @brief Evaluate cost for each batch
   *
   * @param batches_of_trajectories batch of trajectories: tensor of shape [
   * batch_size_, time_steps_, 3 ] where 3 stands for x, y, yaw
   * @return Cost for each batch, tensor of shape [ batch_size ]
   */
  xt::xtensor<T, 1>
  evalBatchesCosts(const xt::xtensor<T, 3> &trajectories,
                   const nav_msgs::msg::Path &global_plan,
                   const geometry_msgs::msg::PoseStamped &robot_pose) const {
    size_t batch_size = trajectories.shape()[0];
    xt::xtensor<T, 1> costs = xt::zeros<T>({batch_size});

    if (global_plan.poses.empty()) {
      return costs;
    }

    xt::xtensor<T, 2> path = std::move(geometry::toTensor<T>(global_plan));
    for (size_t q = 0; q < critics_.size(); q++) {
      critics_[q]->score(robot_pose, trajectories, path, costs);
    }

    return costs;
  }
};

template <typename T>
class GoalCritic : public CriticFunction<T> {
public:
  void
  on_configure() final {}

  GoalCritic(unsigned int power, double weight) {
    power_ = power;
    weight_ = weight;
  }

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  virtual void
  score(const geometry_msgs::msg::PoseStamped &robot_pose,
        const xt::xtensor<T, 3> &trajectories,
        const xt::xtensor<T, 2> &path,
        xt::xtensor<T, 1> &costs) final {
    (void)robot_pose;

    const auto goal_points = xt::view(path, -1, xt::range(0, 2));

    auto last_timestep_points =
        xt::view(trajectories, xt::all(), -1, xt::range(0, 2));

    auto dim = last_timestep_points.dimension() - 1;

    auto &&batches_last_to_goal_dists =
        xt::norm_l2(std::move(last_timestep_points) - goal_points, {dim});

    costs += xt::pow(std::move(batches_last_to_goal_dists) * weight_, power_);
  }

private:
  unsigned int power_{1};
  double weight_{1};
};

template <typename T>
class approxReferenceTrajectoryCritic : public CriticFunction<T> {
public:
  void
  on_configure() final {}

  approxReferenceTrajectoryCritic(unsigned int power, double weight) {
    power_ = power;
    weight_ = weight;
  }

  /**
   * @brief Evaluate cost related to trajectories path alignment using
   * approximate path to segment function
   *
   * @param batches_of_trajectories
   * @param costs [out] add reference cost values to this tensor
   */
  virtual void
  score(const geometry_msgs::msg::PoseStamped &robot_pose,
        const xt::xtensor<T, 3> &trajectories,
        const xt::xtensor<T, 2> &path,
        xt::xtensor<T, 1> &costs) final {
    (void)robot_pose;

    auto path_points = xt::view(path, xt::all(), xt::range(0, 2));
    auto batch_of_lines = xt::view(trajectories, xt::all(), xt::all(),
                                   xt::newaxis(), xt::range(0, 2));

    auto dists = xt::norm_l2(path_points - batch_of_lines,
                             {batch_of_lines.dimension() - 1});
    auto &&cost = xt::mean(xt::amin(std::move(dists), 1), 1);
    costs += xt::pow(std::move(cost) * weight_, power_);
  }

private:
  unsigned int power_{1};
  double weight_{1};
};

template <typename T>
class referenceTrajectoryCritic : public CriticFunction<T> {
public:
  void
  on_configure() final {}

  referenceTrajectoryCritic(unsigned int power, double weight) {
    power_ = power;
    weight_ = weight;
  }

  /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param costs [out] add reference cost values to this tensor
   */
  virtual void
  score(const geometry_msgs::msg::PoseStamped &robot_pose,
        const xt::xtensor<T, 3> &trajectories,
        const xt::xtensor<T, 2> &path,
        xt::xtensor<T, 1> &costs) final {
    (void)robot_pose;

    using xt::evaluation_strategy::immediate;

    xt::xtensor<T, 3> path_to_batches_dists =
        geometry::distPointsToLineSegments2D(path, trajectories);

    xt::xtensor<T, 1> cost = xt::mean(
        xt::amin(std::move(path_to_batches_dists), 1, immediate), 1, immediate);

    costs += xt::pow(std::move(cost) * weight_, power_);
  }

private:
  unsigned int power_{1};
  double weight_{1};
};

template <typename T>
class ObstaclesCritic : public CriticFunction<T> {
public:
  void
  on_configure() final {}

  ObstaclesCritic(double inflation_cost_scaling_factor,
                  double inscribed_radius,
                  double inflation_radius,
                  nav2_costmap_2d::Costmap2DROS *const costmap_ros,
                  unsigned int power,
                  double weight) {
    power_ = power;
    weight_ = weight;
    inflation_cost_scaling_factor_ = inflation_cost_scaling_factor;
    inscribed_radius_ = inscribed_radius;
    inflation_radius_ = inflation_radius;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
  }

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  virtual void
  score(const geometry_msgs::msg::PoseStamped &robot_pose,
        const xt::xtensor<T, 3> &trajectories,
        const xt::xtensor<T, 2> &path,
        xt::xtensor<T, 1> &costs) final {
    (void)robot_pose;
    (void)path;

    size_t batch_size = trajectories.shape()[0];
    size_t time_steps = trajectories.shape()[1];

    constexpr T collision_cost_value = std::numeric_limits<T>::max() / 2;

    auto minDistToObstacle = [this](const auto cost) {
      return (-1.0 / inflation_cost_scaling_factor_) *
                 std::log(static_cast<double>(cost) /
                          (static_cast<double>(
                               nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) -
                           1.0)) +
             inscribed_radius_;
    };

    for (size_t i = 0; i < batch_size; ++i) {
      double min_dist = std::numeric_limits<double>::max();
      bool inflated = false;
      for (size_t j = 0; j < time_steps; ++j) {
        std::array<double, 3> pose = {trajectories(i, j, 0),
                                      trajectories(i, j, 1),
                                      trajectories(i, j, 2)};

        auto footprint =
            getOrientedFootprint(pose, costmap_ros_->getRobotFootprint());

        unsigned char cost =
            static_cast<unsigned char>(scoreFootprint(footprint));

        if (inCollision(cost)) {
          costs[i] = collision_cost_value;
          inflated = false;
          break;
        }

        if (cost != nav2_costmap_2d::FREE_SPACE) {
          min_dist = std::min(minDistToObstacle(cost), min_dist);
          inflated = true;
        }
      }

      if (inflated)
        costs[i] += static_cast<T>(
            pow((1.01 * inflation_radius_ - min_dist) * weight_, power_));
    }
  }

private:
  bool
  inCollision(unsigned char cost) const {
    if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
      return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
             cost != nav2_costmap_2d::NO_INFORMATION;
    } else {
      return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    }
  }

  std::vector<geometry_msgs::msg::Point>
  getOrientedFootprint(
      const std::array<double, 3> &robot_pose,
      const std::vector<geometry_msgs::msg::Point> &footprint_spec) const {
    std::vector<geometry_msgs::msg::Point> oriented_footprint;
    oriented_footprint.resize(footprint_spec.size());

    double cost_yaw = cos(robot_pose[2]);
    double sin_yaw = sin(robot_pose[2]);

    for (size_t i = 0; i < footprint_spec.size(); ++i) {
      oriented_footprint[i].x = robot_pose[0] + footprint_spec[i].x * cost_yaw -
                                footprint_spec[i].y * sin_yaw;
      oriented_footprint[i].y = robot_pose[1] + footprint_spec[i].x * sin_yaw +
                                footprint_spec[i].y * cost_yaw;
    }

    return oriented_footprint;
  }

  double
  lineCost(int x0, int x1, int y0, int y1) const {
    double line_cost = 0.0;
    double point_cost = -1.0;

    for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
      point_cost = static_cast<double>(
          costmap_->getCost(static_cast<unsigned int>(line.getX()),
                            static_cast<unsigned int>(line.getY())));

      if (line_cost < point_cost) {
        line_cost = point_cost;
      }
    }

    return line_cost;
  }

  double
  scoreFootprint(
      const std::vector<geometry_msgs::msg::Point> &footprint) const {
    unsigned int x0, x1, y0, y1;
    double line_cost = 0.0;
    double footprint_cost = 0.0;

    for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
      if (!costmap_->worldToMap(footprint[i].x, footprint[i].y, x0, y0))
        throw std::runtime_error("Footprint Goes Off Grid.");

      if (!costmap_->worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1))
        throw std::runtime_error("Footprint Goes Off Grid.");

      line_cost = lineCost(static_cast<int>(x0), static_cast<int>(x1),
                           static_cast<int>(y0), static_cast<int>(y1));

      footprint_cost = std::max(line_cost, footprint_cost);
    }

    if (!costmap_->worldToMap(footprint.back().x, footprint.back().y, x0, y0))
      throw std::runtime_error("Footprint Goes Off Grid.");

    if (!costmap_->worldToMap(footprint.front().x, footprint.front().y, x1, y1))
      throw std::runtime_error("Footprint Goes Off Grid.");

    line_cost = lineCost(static_cast<int>(x0), static_cast<int>(x1),
                         static_cast<int>(y0), static_cast<int>(y1));

    footprint_cost = std::max(line_cost, footprint_cost);

    return footprint_cost;
  }

  double inflation_cost_scaling_factor_{1};
  double inscribed_radius_{1};
  double inflation_radius_{1};
  unsigned int power_{1};
  double weight_{1};
  nav2_costmap_2d::Costmap2DROS *costmap_ros_;
  nav2_costmap_2d::Costmap2D *costmap_;
};

template <typename T>
class GoalAngleCritic : public CriticFunction<T> {
public:
  void
  on_configure() final {}

  GoalAngleCritic(double threshold_to_consider_goal_angle,
                  unsigned int power,
                  double weight) {
    threshold_to_consider_goal_angle_ = threshold_to_consider_goal_angle;
    power_ = power;
    weight_ = weight;
  }

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  virtual void
  score(const geometry_msgs::msg::PoseStamped &robot_pose,
        const xt::xtensor<T, 3> &trajectories,
        const xt::xtensor<T, 2> &path,
        xt::xtensor<T, 1> &costs) final {
    xt::xtensor<T, 1> tensor_pose = {
        static_cast<T>(robot_pose.pose.position.x),
        static_cast<T>(robot_pose.pose.position.y)};

    auto path_points = xt::view(path, -1, xt::range(0, 2));

    double points_to_goal_dists = xt::norm_l2(tensor_pose - path_points, {0})();

    if (points_to_goal_dists < threshold_to_consider_goal_angle_) {
      auto yaws = xt::view(trajectories, xt::all(), xt::all(), 2);
      auto goal_yaw = xt::view(path, -1, 2);

      costs +=
          xt::pow(xt::mean(xt::abs(yaws - goal_yaw), {1}) * weight_, power_);
    }
  }

private:
  double threshold_to_consider_goal_angle_;
  unsigned int power_{1};
  double weight_{1};
};

}  // namespace mppi::optimization
