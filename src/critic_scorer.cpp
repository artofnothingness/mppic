#include "mppic/critic_scorer.hpp"

namespace mppi::optimization {

void CriticScorer::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & node_name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  node_name_ = node_name;

  getParams();
  setLoader();
  loadCritics();
  configureCritics();

  RCLCPP_INFO(logger_, "Configured");
}

void CriticScorer::setLoader()
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<optimization::CriticFunction>>(
      "mppic", getFullName(base_name_));
  }
}

std::string CriticScorer::getFullName(const std::string & name)
{
  return "mppi::optimization::" + name;
}

void CriticScorer::getParams()
{
  auto node = parent_.lock();
  auto getParam = utils::getParamGetter(node, node_name_);
  getParam(critics_names_, "critics_names", std::vector<std::string>{});
}

void CriticScorer::loadCritics()
{
  critics_.clear();
  for (auto name : critics_names_) {
    std::string fullname = getFullName(name);
    auto instance =
      std::unique_ptr<optimization::CriticFunction>(loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }
}

void CriticScorer::configureCritics()
{
  for (size_t q = 0; q < critics_.size(); q++) {
    critics_[q]->on_configure(parent_, node_name_ + "." + critics_names_[q], costmap_ros_);
  }
}

xt::xtensor<double, 1> CriticScorer::evalTrajectoriesScores(
  const xt::xtensor<double, 3> & trajectories, const nav_msgs::msg::Path & global_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  size_t trajectories_count = trajectories.shape()[0];
  xt::xtensor<double, 1> costs = xt::zeros<double>({trajectories_count});

  if (global_plan.poses.empty()) {
    return costs;
  }

  xt::xtensor<double, 2> path = std::move(utils::toTensor<double>(global_plan));

  for (size_t q = 0; q < critics_.size(); q++) {
    critics_[q]->score(robot_pose, trajectories, path, costs);
  }

  return costs;
}

} // namespace mppi::optimization
