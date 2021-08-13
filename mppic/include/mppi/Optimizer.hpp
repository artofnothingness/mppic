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

template <typename T, typename Tensor = xt::xarray<T>,
          typename Model = Tensor(const Tensor &)>
class Optimizer {

public:
  Optimizer() = default;
  ~Optimizer() = default;

  Optimizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
            const std::string &node_name, nav2_costmap_2d::Costmap2D *costmap,
            Model &&model)
      : model_(model) {

    node_name_ = node_name;
    parent_ = parent;
    costmap_ = costmap;
  }

  void on_configure() {
    getParams();
    resetBatches();
    RCLCPP_INFO(logger_, "Configured");
  }

  void on_cleanup(){};
  void on_activate(){};
  void on_deactivate(){};

  auto evalNextControl(const geometry_msgs::msg::Twist &twist,
                       const nav_msgs::msg::Path &path)
      -> geometry_msgs::msg::TwistStamped;

  auto getTrajectories() -> Tensor { return trajectories_; }

private:
  void getParams();
  void resetBatches();

  auto generateNoisedTrajectoryBatches(const geometry_msgs::msg::Twist &twist)
      -> Tensor;
  auto generateNoisedControlBatches() -> Tensor;
  void applyControlConstraints();
  void setBatchesVelocity(const geometry_msgs::msg::Twist &twist);
  void setBatchesInitialVelocities(const geometry_msgs::msg::Twist &twist);
  void propagateBatchesVelocityFromInitials();
  auto integrateVelocityBatches() const -> Tensor;
  auto evalBatchesCosts(const Tensor &trajectory_batches,
                        const nav_msgs::msg::Path &path) const -> Tensor;

  void updateControlSequence(Tensor &costs);

  template <typename H>
  auto getControlFromSequence(const H &header)
      -> geometry_msgs::msg::TwistStamped;

  decltype(auto) getControlBatches() {
    return xt::view(batches_, xt::all(), xt::all(), xt::range(2, 4));
  }

  decltype(auto) getLinearVelocityControlBatches() {
    return xt::view(batches_, xt::all(), xt::all(), 2);
  }

  decltype(auto) getAngularVelocityControlBatches() {
    return xt::view(batches_, xt::all(), xt::all(), 3);
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::string node_name_;
  nav2_costmap_2d::Costmap2D *costmap_;

  static constexpr int last_dim_ = 5;
  static constexpr int control_dim_size_ = 2;

  int batch_size_;
  int time_steps_;
  int iteration_count_;

  double model_dt_;
  double std_v_;
  double std_w_;
  double limit_v_;
  double limit_w_;
  double temperature_;

  Tensor batches_;
  Tensor control_sequence_;
  Tensor trajectories_;

  std::function<Model> model_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPI Optimizer")};
};

} // namespace mppi::optimization
