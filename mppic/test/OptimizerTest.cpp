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

    std::string node_name = "Test";
    costmap_ros_ = std::make_shared<Costmap2DROS>("cost_map_node");
    node_ = std::make_shared<LifecycleNode>(node_name);

    auto &model = models::NaiveModel<T>;
    auto costmap = costmap_ros_->getCostmap();

    optimizer_ = Optimizer(node_, node_name, costmap, model);
  }

  void TearDown() override {}

protected:
  shared_ptr<LifecycleNode> node_;
  Optimizer optimizer_;
  shared_ptr<Costmap2DROS> costmap_ros_;
};

TEST_F(OptimizerTest, evalNextControlTest) {
  PoseStamped pose;
  Twist twist;
  TwistStamped result;
  Path path;

  result = optimizer_.evalNextControl(pose, twist, path);

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
