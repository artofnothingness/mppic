#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "xtensor/xarray.hpp"
#include <xtensor/xview.hpp>

namespace mppi::optimization {

template <typename T,
          typename Tensor = xt::xarray<T>,
          typename Model = Tensor(const Tensor &)>
class Optimizer {
public:
  Optimizer() = default;
  ~Optimizer() = default;

  Optimizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
            const std::string &node_name,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros,
            Model &&model)
      : parent_(parent),
        node_name_(node_name),
        costmap_ros_(costmap_ros),
        model_(model) {}

  void on_configure();
  void on_cleanup(){};
  void on_activate(){};
  void on_deactivate(){};

  /**
   * @brief Evaluate current best control
   *
   * @return best control
   */
  auto evalNextControl(const geometry_msgs::msg::PoseStamped &robot_pose,
                       const geometry_msgs::msg::Twist &robot_speed,
                       const nav_msgs::msg::Path &plan)
      -> geometry_msgs::msg::TwistStamped;

  auto getGeneratedTrajectories() -> Tensor { return generated_trajectories_; }

private:
  void getParams();
  void resetBatches();

  /**
   * @brief Invoke generateNoisedControlBatches, assign result to batches controls 
   * and integrate recieved controls in trajectories
   *
   * @return trajectories Tensor of shape [ batch_size_, time_steps_, 3]  where 3 stands for x, y, yaw
   */
  auto generateNoisedTrajectories(const geometry_msgs::msg::PoseStamped &robot_pose,
                                  const geometry_msgs::msg::Twist &robot_speed)
      -> Tensor;

  /**
   * @brief Generate random controls by gaussian noise with mean in
   * control_sequence_
   *
   * @return Control batches
   */
  auto generateNoisedControlBatches() -> Tensor;

  void applyControlConstraints();

  /**
   * @brief Invoke setBatchesInitialVelocities and propagateBatchesVelocitiesFromInitials
   *
   * @param twist current robot speed
   */
  void setBatchesVelocities(const geometry_msgs::msg::Twist &twist);

  void setBatchesInitialVelocities(const geometry_msgs::msg::Twist &twist);

  /**
   * @brief propagate velocities in batches_ using model
   * for time_steps_ time horizont
   *
   */
  void propagateBatchesVelocitiesFromInitials();

  auto integrateBatchesVelocities(const geometry_msgs::msg::PoseStamped &pose) 
    const -> Tensor;



  /**
   * @brief Evaluate cost for every batch
   *
   * @param trajectory_batches batch of trajectories: 
   * Tensor of shape [batch_size_, time_steps_, 3] where 3 stands for x, y, yaw
   * @param path global path
   * @return batches costs: Tensor of shape [batch_size]
   */
  auto evalBatchesCosts(const Tensor &batches_of_trajectories,
                        const nav_msgs::msg::Path &path) const -> Tensor;
 
  /**
   * @brief Evaluate cost related to distances between generated 
   * and reference trajectories
   *
   * @tparam D type of tensor consisting distances between points of generated 
   * and reference trajectories
   * @param dists distances between points of generated 
   * and reference trajectories: Tensor of shape [ batch_size_, time_steps_, point_size ]
   * @return batches costs: type of shape [ batch_size_ ]
   */
  template<typename D>
  auto evalReferenceCost(const D &dists) const;

  /**
   * @brief Evaluate cost related to distances between last path 
   * points and batch trajectories points on last time step
   *
   * @tparam P tensor-like type of path points
   * @tparam L tensor-like type of batch_points
   * @param path_points tensor-like type of shape [ path points count, 2 ] 
   * where 2 stands for x, y
   * @param batchs_of_trajectories_points tensor-like type of shape [ batch, time_steps_ 2 ]
   * where 2 stands for x, y 
   * @return batches costs: type of shape [ batch_size_ ]
   */
  template <typename L, typename P>
  auto evalGoalCost(const P &path_points, const L &batchs_of_trajectories_points) const;

  template <typename L>
  auto evalObstacleCost(const L &batchs_of_trajectories_points) const;

  T costAtPose(const double & x, const double & y) const;

  bool inCollision(unsigned char cost) const;

  /**
   * @brief Update control_sequence_ with weighted by costs batch controls
   *
   * @param costs batches costs
   */
  void updateControlSequence(const Tensor &costs);

  /**
   * @brief Get first control from control_sequence_
   *
   */

template <typename S>
auto getControlFromSequence(const S &stamp, const std::string &frame)
      -> geometry_msgs::msg::TwistStamped;

  auto getBatchesControls() const;
  auto getBatchesControls();
  auto getBatchesControlLinearVelocities() const;
  auto getBatchesControlLinearVelocities();
  auto getBatchesControlAngularVelocities() const;
  auto getBatchesControlAngularVelocities();
  auto getBatchesLinearVelocities() const;

  auto getBatchesLinearVelocities();
  auto getBatchesAngularVelocities() const;
  auto getBatchesAngularVelocities();

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::string node_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::function<Model> model_;

  static constexpr int last_dim_size = 5;
  static constexpr int control_dim_size_ = 2;

  int batch_size_;
  int time_steps_;
  int iteration_count_;

  double model_dt_;
  double v_std_;
  double w_std_;
  double v_limit_;
  double w_limit_;
  double temperature_;

  size_t reference_cost_power_;
  size_t reference_cost_weight_;
  size_t goal_cost_power_;
  size_t goal_cost_weight_;
  size_t obstacle_cost_power_;
  size_t obstacle_cost_weight_;

  Tensor batches_;
  Tensor control_sequence_;
  Tensor generated_trajectories_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPI Optimizer")};
};

} // namespace mppi::optimization
