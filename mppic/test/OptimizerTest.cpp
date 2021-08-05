#include <gtest/gtest.h>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <rclcpp/executors.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "mppi/Models.hpp"
#include "mppi/Optimizer.hpp"

using namespace ultra::mppi;

using T = float;
using Optimizer = optimization::Optimizer<T>;
using nav2_costmap_2d::Costmap2DROS;
using rclcpp_lifecycle::LifecycleNode;
using std::shared_ptr;

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Path;

class OptimizerTest : public ::testing::Test {
protected:

  void SetUp() override {

    costmap_ros_ = std::make_shared<Costmap2DROS>("cost_map_node");

    auto optimizer = [&] {
      auto &model = models::NaiveModel<T>;
      double model_dt = 0.1;
      int time_steps = 20;
      int batch_size = 100;
      double std_v = 0.1;
      double std_w = 0.1;
      double limit_v = 0.5;
      double limit_w = 1.0;
      int iteration_count = 2;
      double temperature = 0.25;
      return Optimizer(batch_size, std_v, std_w, limit_v, limit_w, model_dt,
                       time_steps, iteration_count, temperature, model);
    }();
  }

  void TearDown() override {}

protected:
  Optimizer optimizer_;
  shared_ptr<Costmap2DROS> costmap_ros_;
};

TEST_F(OptimizerTest, evalNextControlTest) {
  PoseStamped pose;
  Twist twist;
  TwistStamped result;
  Path path;

  auto &costmap = *(costmap_ros_->getCostmap());

  result = optimizer_.evalNextControl(pose, twist, path, costmap);
  EXPECT_TRUE(result == TwistStamped{});
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  using namespace std::string_literals;
  auto node = std::make_shared<LifecycleNode>("TestNode");
  ::testing::InitGoogleTest(&argc, argv);
  auto res = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return res;
}
