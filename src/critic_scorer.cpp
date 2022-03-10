#include "mppic/critic_scorer.hpp"

namespace mppi
{

template<typename T>
void CriticScorer<T>::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  const std::string & node_name,
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

template<typename T>
void CriticScorer<T>::setLoader()
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction<T>>>(
      "mppic", getFullName(base_name_));
  }
}

template<typename T>
std::string CriticScorer<T>::getFullName(const std::string & name)
{
  return "mppi::critics::" + name + "<" + critics_type_ + ">";
}

template<typename T>
void CriticScorer<T>::getParams()
{
  auto node = parent_.lock();
  auto getParam = utils::getParamGetter(node, node_name_);
  getParam(critics_names_, "critics_names", std::vector<std::string>{});
  getParam(critics_type_, "critics_type", std::string("float"));
}

template<typename T>
void CriticScorer<T>::loadCritics()
{
  critics_.clear();
  for (auto name : critics_names_) {
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction<T>>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }
}

template<typename T>
void CriticScorer<T>::configureCritics()
{
  for (size_t q = 0; q < critics_.size(); q++) {
    critics_[q]->on_configure(parent_, node_name_ + "." + critics_names_[q], costmap_ros_);
  }
}

template<typename T>
xt::xtensor<T, 1> CriticScorer<T>::evalTrajectoriesScores(
  const xt::xtensor<T, 3> & trajectories, const nav_msgs::msg::Path & global_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  size_t trajectories_count = trajectories.shape()[0];
  xt::xtensor<T, 1> costs = xt::zeros<T>({trajectories_count});

  if (global_plan.poses.empty()) {
    return costs;
  }

  xt::xtensor<T, 2> path = std::move(utils::toTensor<T>(global_plan));

  for (size_t q = 0; q < critics_.size(); q++) {
    critics_[q]->score(robot_pose, trajectories, path, costs);
  }

  return costs;
}

} // namespace mppi
