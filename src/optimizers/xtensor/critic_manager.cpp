// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/optimizers/xtensor/critic_manager.hpp"

#include "mppic/context.hpp"

namespace mppi::xtensor
{

void CriticManager::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, ParametersHandler * param_handler)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  auto node = parent_.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  getParams();
  loadCritics();
}

void CriticManager::getParams()
{
  auto node = parent_.lock();
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
  getParam(profile_, "profile", false);
}

void CriticManager::loadCritics()
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
      "mppic", "mppi::xtensor::critics::CriticFunction");
  }

  critics_.clear();
  for (auto name : critic_names_) {
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(parent_, name_ + "." + name, costmap_ros_, parameters_handler_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::xtensor::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  models::CriticFunctionData & data) const
{
  for (size_t q = 0; q < critics_.size(); q++) {
    if constexpr (context::profile) {
      auto start = std::chrono::high_resolution_clock::now();
      critics_[q]->score(data);
      auto stop = std::chrono::high_resolution_clock::now();
      size_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 500, "Critic %s score take: %ld [ms]",
        critics_[q]->getName().c_str(), duration);
    } else {
      critics_[q]->score(data);
    }
  }

}

}  // namespace mppi::xtensor
