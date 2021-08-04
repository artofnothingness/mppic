#include <gtest/gtest.h>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <rclcpp/executors.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "mppi/Optimizer.hpp" 
#include "mppi/Models.hpp"

using namespace ultra::mppi;


class OptimizerTest : public ::testing::Test {
protected:
  using T = float;
  using ManagedNode = rclcpp_lifecycle::LifecycleNode;
  using Optimizer = optimization::Optimizer<T>;
  using Costmap2DROS = nav2_costmap_2d::Costmap2DROS;
  using TfBuffer = tf2_ros::Buffer;


  void SetUp() override {
    using namespace std::string_literals;

    auto node_name = "TestNode"s;
    auto node = std::make_shared<ManagedNode>(node_name);
    auto cost_map = std::make_shared<Costmap2DROS>("cost_map_node");
    auto tf_buffer = std::shared_ptr<TfBuffer>();

    auto &model = models::NaiveModel<T>;

    auto optimizer = Optimizer(node, node_name, tf_buffer, cost_map, model);
  }

  void TearDown() override {}
  Optimizer optimizer;
};


TEST_F(OptimizerTest, evalNextControlTest) {

  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist twist;
  geometry_msgs::msg::TwistStamped result;
  
  result = optimizer.evalNextControl(pose, twist);
  EXPECT_TRUE( result == geometry_msgs::msg::TwistStamped{} );
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return res;
}
