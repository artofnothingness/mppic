#pragma once


#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <xtensor/xarray.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xview.hpp>

#include "mppic/utils/LineIterator.hpp"
#include "mppic/utils/common.hpp"
#include "mppic/utils/geometry.hpp"

namespace mppi::optimization {
// TODO pluginize
// TODO separate core from critics
template<typename T>
class CriticFunction
{
public:
  CriticFunction() = default;
  virtual ~CriticFunction() = default;

  void on_configure(rclcpp_lifecycle::LifecycleNode *const parent,
    const std::string &parent_name,
    const std::string &component_name,
    nav2_costmap_2d::Costmap2DROS *const costmap_ros)
  {
    parent_ = parent;
    node_name_ = parent_name + "." + component_name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    getParams();
  }

  virtual void getParams() = 0;

  virtual void score(const geometry_msgs::msg::PoseStamped &robot_pose,
    const xt::xtensor<T, 3> &trajectories,
    const xt::xtensor<T, 2> &path,
    xt::xtensor<T, 1> &costs) = 0;

protected:
  rclcpp_lifecycle::LifecycleNode *parent_;
  std::string node_name_;
  nav2_costmap_2d::Costmap2DROS *costmap_ros_;
  nav2_costmap_2d::Costmap2D *costmap_;
};

template<typename T>
class CriticScorer
{
public:
  CriticScorer() = default;
  explicit CriticScorer(std::vector<std::unique_ptr<CriticFunction<T>>> &&critics)
    : critics_(std::move(critics))
  {}

  void on_configure(rclcpp_lifecycle::LifecycleNode *const parent,
    const std::string &parent_name,
    const std::string &component_name,
    nav2_costmap_2d::Costmap2DROS *const costmap_ros)
  {
    for (size_t q = 0; q < critics_.size(); q++) {
      critics_[q]->on_configure(parent, parent_name, component_name, costmap_ros);
    }
  }

  /**
   * @brief Evaluate cost for each trajectory
   *
   * @param trajectories: tensor of shape [ ..., ..., 3 ]
   * where 3 stands for x, y, yaw
   * @return Cost for each trajectory
   */
  xt::xtensor<T, 1> evalTrajectoriesScores(const xt::xtensor<T, 3> &trajectories,
    const nav_msgs::msg::Path &global_plan,
    const geometry_msgs::msg::PoseStamped &robot_pose) const
  {
    size_t trajectories_count = trajectories.shape()[0];
    xt::xtensor<T, 1> costs = xt::zeros<T>({ trajectories_count });

    if (global_plan.poses.empty()) { return costs; }

    xt::xtensor<T, 2> path = std::move(geometry::toTensor<T>(global_plan));

    for (size_t q = 0; q < critics_.size(); q++) {
      critics_[q]->score(robot_pose, trajectories, path, costs);
    }

    return costs;
  }

private:
  std::vector<std::unique_ptr<CriticFunction<T>>> critics_;
};

template<typename T>
class GoalCritic : public CriticFunction<T>
{
public:
  void getParams() final
  {
    auto getParam = utils::getParamGetter(this->parent_, this->node_name_);
    getParam(power_, "goal_cost_power", 1);
    getParam(weight_, "goal_cost_weight", 20);
  }

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  virtual void score(const geometry_msgs::msg::PoseStamped &robot_pose,
    const xt::xtensor<T, 3> &trajectories,
    const xt::xtensor<T, 2> &path,
    xt::xtensor<T, 1> &costs) final
  {
    (void)robot_pose;

    const auto goal_points = xt::view(path, -1, xt::range(0, 2));

    auto trajectories_end = xt::view(trajectories, xt::all(), -1, xt::range(0, 2));

    auto dim = trajectories_end.dimension() - 1;

    auto &&dists_trajectories_end_to_goal =
      xt::norm_l2(std::move(trajectories_end) - goal_points, { dim });

    costs += xt::pow(std::move(dists_trajectories_end_to_goal) * weight_, power_);
  }

private:
  unsigned int power_{ 0 };
  double weight_{ 0 };
};

template<typename T>
class approxReferenceTrajectoryCritic : public CriticFunction<T>
{
public:
  void getParams() final
  {
    auto getParam = utils::getParamGetter(this->parent_, this->node_name_);
    getParam(power_, "reference_cost_power", 1);
    getParam(weight_, "reference_cost_weight", 20);
  }

  /**
   * @brief Evaluate cost related to trajectories path alignment using
   * approximate path to segment function
   *
   * @param costs [out] add reference cost values to this tensor
   */
  virtual void score(const geometry_msgs::msg::PoseStamped &robot_pose,
    const xt::xtensor<T, 3> &trajectories,
    const xt::xtensor<T, 2> &path,
    xt::xtensor<T, 1> &costs) final
  {
    (void)robot_pose;

    auto path_points = xt::view(path, xt::all(), xt::range(0, 2));
    auto trajectories_points_extended =
      xt::view(trajectories, xt::all(), xt::all(), xt::newaxis(), xt::range(0, 2));

    auto dists = xt::norm_l2(
      path_points - trajectories_points_extended, { trajectories_points_extended.dimension() - 1 });
    auto &&cost = xt::mean(xt::amin(std::move(dists), 1), 1);
    costs += xt::pow(std::move(cost) * weight_, power_);
  }

private:
  unsigned int power_{ 0 };
  double weight_{ 0 };
};

template<typename T>
class referenceTrajectoryCritic : public CriticFunction<T>
{
public:
  void getParams() final
  {
    auto getParam = utils::getParamGetter(this->parent_, this->node_name_);
    getParam(power_, "reference_cost_power", 1);
    getParam(weight_, "reference_cost_weight", 20);
  }

  /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param costs [out] add reference cost values to this tensor
   */
  virtual void score(const geometry_msgs::msg::PoseStamped &robot_pose,
    const xt::xtensor<T, 3> &trajectories,
    const xt::xtensor<T, 2> &path,
    xt::xtensor<T, 1> &costs) final
  {
    (void)robot_pose;

    using xt::evaluation_strategy::immediate;

    xt::xtensor<T, 3> dists_path_to_trajectories =
      geometry::distPointsToLineSegments2D(path, trajectories);

    xt::xtensor<T, 1> cost =
      xt::mean(xt::amin(std::move(dists_path_to_trajectories), 1, immediate), 1, immediate);

    costs += xt::pow(std::move(cost) * weight_, power_);
  }

private:
  unsigned int power_{ 0 };
  double weight_{ 0 };
};

template<typename T>
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
  virtual void score(const geometry_msgs::msg::PoseStamped &robot_pose,
    const xt::xtensor<T, 3> &trajectories,
    const xt::xtensor<T, 2> &path,
    xt::xtensor<T, 1> &costs) final
  {
    (void)robot_pose;
    (void)path;

    size_t trajectories_count = trajectories.shape()[0];
    size_t time_steps = trajectories.shape()[1];

    constexpr T collision_cost_value = std::numeric_limits<T>::max() / 2;

    auto minDistToObstacle = [this](const auto cost) {
      return (-1.0 / inflation_cost_scaling_factor_)
               * std::log(
                 static_cast<double>(cost)
                 / (static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) - 1.0))
             + inscribed_radius_;
    };

    for (size_t i = 0; i < trajectories_count; ++i) {
      double min_dist = std::numeric_limits<double>::max();
      bool inflated = false;
      for (size_t j = 0; j < time_steps; ++j) {
        std::array<double, 3> pose = {
          trajectories(i, j, 0), trajectories(i, j, 1), trajectories(i, j, 2)
        };

        auto footprint = getOrientedFootprint(pose, this->costmap_ros_->getRobotFootprint());
        auto cost = static_cast<unsigned char>(scoreFootprint(footprint));

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

      if (inflated) {
        costs[i] += static_cast<T>(pow((1.01 * inflation_radius_ - min_dist) * weight_, power_));
      }
    }
  }

private:
  bool inCollision(unsigned char cost) const
  {
    if (this->costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
      return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE
             && cost != nav2_costmap_2d::NO_INFORMATION;
    }

    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  std::vector<geometry_msgs::msg::Point> getOrientedFootprint(
    const std::array<double, 3> &robot_pose,
    const std::vector<geometry_msgs::msg::Point> &footprint_spec) const
  {
    std::vector<geometry_msgs::msg::Point> oriented_footprint;
    oriented_footprint.resize(footprint_spec.size());

    double cost_yaw = cos(robot_pose[2]);
    double sin_yaw = sin(robot_pose[2]);

    for (size_t i = 0; i < footprint_spec.size(); ++i) {
      oriented_footprint[i].x =
        robot_pose[0] + footprint_spec[i].x * cost_yaw - footprint_spec[i].y * sin_yaw;
      oriented_footprint[i].y =
        robot_pose[1] + footprint_spec[i].x * sin_yaw + footprint_spec[i].y * cost_yaw;
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

      if (line_cost < point_cost) { line_cost = point_cost; }
    }

    return line_cost;
  }

  double scoreFootprint(const std::vector<geometry_msgs::msg::Point> &footprint) const
  {
    unsigned int x0{ 0 };
    unsigned int x1{ 0 };
    unsigned int y0{ 0 };
    unsigned int y1{ 0 };

    double line_cost = 0.0;
    double footprint_cost = 0.0;

    auto world_to_map = [&](size_t i, unsigned int &x, unsigned int &y) {
      if (!this->costmap_->worldToMap(footprint[i].x, footprint[i].y, x, y)) {
        throw std::runtime_error("Footprint Goes Off Grid.");
      }
    };

    for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
      world_to_map(i, x0, y0);
      world_to_map(i + 1, x1, y1);

      line_cost = lineCost(
        static_cast<int>(x0), static_cast<int>(x1), static_cast<int>(y0), static_cast<int>(y1));

      footprint_cost = std::max(line_cost, footprint_cost);
    }

    world_to_map(0, x0, y0);
    world_to_map(footprint.size() - 1, x1, y1);

    line_cost = lineCost(
      static_cast<int>(x0), static_cast<int>(x1), static_cast<int>(y0), static_cast<int>(y1));

    footprint_cost = std::max(line_cost, footprint_cost);

    return footprint_cost;
  }

  double inflation_cost_scaling_factor_{ 0 };
  double inscribed_radius_{ 0 };
  double inflation_radius_{ 0 };
  unsigned int power_{ 0 };
  double weight_{ 0 };
};

template<typename T>
class GoalAngleCritic : public CriticFunction<T>
{
public:
  void getParams() final
  {
    auto getParam = utils::getParamGetter(this->parent_, this->node_name_);
    getParam(power_, "goal_angle_cost_power", 1);
    getParam(weight_, "goal_angle_cost_weight", 10);
    getParam(threshold_to_consider_goal_angle_, "threshold_to_consider_goal_angle", 0.30);
  }

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  virtual void score(const geometry_msgs::msg::PoseStamped &robot_pose,
    const xt::xtensor<T, 3> &trajectories,
    const xt::xtensor<T, 2> &path,
    xt::xtensor<T, 1> &costs) final
  {
    xt::xtensor<T, 1> tensor_pose = { static_cast<T>(robot_pose.pose.position.x),
      static_cast<T>(robot_pose.pose.position.y) };

    auto path_points = xt::view(path, -1, xt::range(0, 2));

    double points_to_goal_dists = xt::norm_l2(tensor_pose - path_points, { 0 })();

    if (points_to_goal_dists < threshold_to_consider_goal_angle_) {
      auto yaws = xt::view(trajectories, xt::all(), xt::all(), 2);
      auto goal_yaw = xt::view(path, -1, 2);

      costs += xt::pow(xt::mean(xt::abs(yaws - goal_yaw), { 1 }) * weight_, power_);
    }
  }

private:
  double threshold_to_consider_goal_angle_{ 0 };
  unsigned int power_{ 0 };
  double weight_{ 0 };
};

}// namespace mppi::optimization
