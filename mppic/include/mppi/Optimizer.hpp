#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "xtensor/xarray.hpp"
#include <xtensor/xview.hpp>

namespace mppi::optimization {

template <typename T, typename Tensor = xt::xarray<T>, typename Model = Tensor(const Tensor &)>
class Optimizer {
public:
  Optimizer() = default;
  ~Optimizer() = default;

  Optimizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
            const std::string &node_name,
            nav2_costmap_2d::Costmap2D *costmap,
            Model &&model)
      : parent_(parent), node_name_(node_name), costmap_(costmap), model_(model) {}

  void on_configure();

  void on_cleanup(){};
  void on_activate(){};
  void on_deactivate(){};

  auto evalNextControl(const geometry_msgs::msg::Twist &twist, const nav_msgs::msg::Path &path)
      -> geometry_msgs::msg::TwistStamped;

  auto getGeneratedTrajectories() -> Tensor { return generated_trajectories_; }

private:
  void getParams();
  void resetBatches();

  auto generateNoisedTrajectories(const geometry_msgs::msg::Twist &twist) -> Tensor;
  auto generateNoisedControlBatches() -> Tensor;
  void applyControlConstraints();
  void setBatchesVelocities(const geometry_msgs::msg::Twist &twist);
  void setBatchesInitialVelocities(const geometry_msgs::msg::Twist &twist);
  void propagateBatchesVelocitiesFromInitials();
  auto integrateBatchesVelocities() const -> Tensor;
  auto evalBatchesCosts(const Tensor &trajectory_batches, const nav_msgs::msg::Path &path) const
      -> Tensor;

  void updateControlSequence(const Tensor &costs);

  template <typename H>
  auto getControlFromSequence(const H &header) -> geometry_msgs::msg::TwistStamped;

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
  nav2_costmap_2d::Costmap2D *costmap_;
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

  Tensor batches_;
  Tensor control_sequence_;
  Tensor generated_trajectories_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPI Optimizer")};
};

} // namespace mppi::optimization
