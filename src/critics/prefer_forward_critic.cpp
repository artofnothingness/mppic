// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#include "mppic/critics/prefer_forward_critic.hpp"
#include <xtensor/xvectorize.hpp>

namespace mppi::critics
{

void PreferForwardCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.0);

  RCLCPP_INFO(
    logger_, "PreferForwardCritic instantiated with %d power and %f weight.", power_, weight_);
}

// Derivative of https://gist.github.com/volkansalma/2972237
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
// |error| < 0.005
float atan2_approx(float y, float x)
{
  if (x == 0.0f) {
    if (y > 0.0f) {
      return PIBY2_FLOAT;
    }
    if (y == 0.0f) {
      return 0.0f;
    }
    return -PIBY2_FLOAT;
  }

  float atan;
  float z = y / x;

  if (fabs(z) < 1.0f ) {
    atan = z / (1.0f + 0.28f * z * z);
    if (x < 0.0f) {
      if (y < 0.0f) {
        return atan - PI_FLOAT;
      }
      return atan + PI_FLOAT;
    }
  } else {
    atan = PIBY2_FLOAT - z / (z * z + 0.28f);
    if (y < 0.0f) {
      return atan - PI_FLOAT;
    }
  }

  return atan;
}

inline auto approx_atan2 = xt::vectorize(atan2_approx);

void PreferForwardCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  using namespace xt::placeholders;  // NOLINT

  auto dx = xt::view(data.trajectories, xt::all(), xt::range(1, _), 0) -
    xt::view(data.trajectories, xt::all(), xt::range(_, -1), 0);
  auto dy = xt::view(data.trajectories, xt::all(), xt::range(1, _), 1) -
    xt::view(data.trajectories, xt::all(), xt::range(_, -1), 1);
  
  auto thetas = approx_atan2(dy, dx);  // Speeds up
  auto yaws = xt::view(data.trajectories, xt::all(), xt::range(_, -1), 2);

  auto yaws_local = thetas - yaws;

  auto forward_translation_reversed = -xt::cos(yaws_local) * xt::hypot(dx, dy);
  
  auto backward_translation = xt::maximum(forward_translation_reversed, 0);

  data.costs += xt::pow(xt::sum(backward_translation, {1}, xt::evaluation_strategy::immediate) * weight_, power_);
}
}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PreferForwardCritic,
  mppi::critics::CriticFunction)
