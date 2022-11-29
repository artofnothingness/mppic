// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/critics/constraint_critic.hpp"

namespace mppi::critics
{

void ConstraintCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0);
  RCLCPP_INFO(
    logger_, "ConstraintCritic instantiated with %d power and %f weight.",
    power_, weight_);

  float vx_max, vy_max, vx_min, vy_min;
  getParentParam(vx_max, "vx_max", 0.5);
  getParentParam(vy_max, "vy_max", 0.0);
  getParentParam(vx_min, "vx_min", -0.35);
  getParentParam(vy_min, "vy_min", 0.0);

  const float min_sgn = vx_min > 0.0 ? 1.0 : -1.0;
  max_vel_ = std::sqrt(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * std::sqrt(vx_min * vx_min + vy_min * vy_min);
}

void ConstraintCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_) {
    return;
  }

  auto out_of_max_bounds_motion = xt::maximum(data.state.vx - max_vel_, 0);
  auto out_of_min_bounds_motion = xt::maximum(min_vel_ - data.state.vx, 0);

  auto acker = dynamic_cast<AckermannMotionModel *>(data.motion_model.get());
  if (acker != nullptr) {
    auto & vx = data.state.vx;
    auto & wz = data.state.wz;
    auto out_of_turning_rad_motion = xt::maximum(
      acker->getMinTurningRadius() - (xt::fabs(vx) / xt::fabs(wz)), 0.0);

    data.costs += xt::pow(
      xt::sum(
        (std::move(out_of_max_bounds_motion) +
        std::move(out_of_min_bounds_motion) +
        std::move(out_of_turning_rad_motion)) *
        data.model_dt, {1}, immediate) * weight_, power_);
  }

  data.costs += xt::pow(
    xt::sum(
      (std::move(out_of_max_bounds_motion) +
      std::move(out_of_min_bounds_motion)) *
      data.model_dt, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ConstraintCritic, mppi::critics::CriticFunction)
