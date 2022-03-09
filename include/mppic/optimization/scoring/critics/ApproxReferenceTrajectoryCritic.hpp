#pragma once

#include <xtensor/xmath.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "mppic/optimization/scoring/CriticFunction.hpp"
#include "mppic/utils.hpp"

namespace mppi::optimization {

template <typename T>
class ApproxReferenceTrajectoryCritic : public CriticFunction<T>
{
public:
  using CriticFunction<T>::parent_;
  using CriticFunction<T>::node_name_;

  void getParams() final
  {
    auto getParam = utils::getParamGetter(parent_, node_name_);
    getParam(power_, "reference_cost_power", 1);
    getParam(weight_, "reference_cost_weight", 15);
  }

  /**
   * @brief Evaluate cost related to trajectories path alignment using
   * approximate path to segment function
   *
   * @param costs [out] add reference cost values to this tensor
   */
  virtual void score(
    const geometry_msgs::msg::PoseStamped & robot_pose, const xt::xtensor<T, 3> & trajectories,
    const xt::xtensor<T, 2> & path, xt::xtensor<T, 1> & costs) final
  {
    (void)robot_pose;

    auto path_points = xt::view(path, xt::all(), xt::range(0, 2));
    auto trajectories_points_extended =
      xt::view(trajectories, xt::all(), xt::all(), xt::newaxis(), xt::range(0, 2));

    auto dists = xt::norm_l2(
      path_points - trajectories_points_extended, {trajectories_points_extended.dimension() - 1});
    auto && cost = xt::mean(xt::amin(std::move(dists), 1), 1);
    costs += xt::pow(std::move(cost) * weight_, power_);
  }

private:
  unsigned int power_{0};
  double weight_{0};
};

} // namespace mppi::optimization
