#include <gtest/gtest.h>
#include <rclcpp/executors.hpp>

#include "mppi/Optimizer.hpp" 

using namespace ultra::mppi;


class OptimizerTest : public ::testing::Test {
protected:
  using ManagedNode = rclcpp_lifecycle::LifecycleNode;
  using Optimizer = optimization::Optimizer<float>;

  void SetUp() override {
    auto node = std::make_shared<ManagedNode>("TestNode");
    optimizer = Optimizer();

    optimizer.m_model_dt = 0.1;
    optimizer.m_time_steps = 20;
    optimizer.m_batch_size = 100;
    optimizer.m_std_v = 0.1;
    optimizer.m_std_w = 0.1;
    optimizer.m_limit_v = 0.5;
    optimizer.m_limit_w = 1.0;
    optimizer.m_iterations_count = 2;
    optimizer.m_lookahead_dist = 1.2;
    optimizer.reset();

  }

  void TearDown() override {}

  Optimizer optimizer;
};


TEST_F(OptimizerTest, SetUpBatchShapeTest) {
    auto shape = optimizer.m_batches.shape();

    std::vector<int> expected_shape = {
      optimizer.m_batch_size, 
      optimizer.m_time_steps, 
      optimizer.m_last_dim
    };

    for (unsigned int i = 0; i < shape.size(); ++i) {
      EXPECT_EQ(shape[i], static_cast<unsigned int>(expected_shape[i])) << "Shapes differ at index " << i;
    }
}


TEST_F(OptimizerTest, SetUpControlSeqShapeTest) {
    auto shape = optimizer.m_control_sequence.shape();

    std::vector<int> expected_shape = {
      optimizer.m_time_steps, 
      optimizer.m_control_dim, 
    };

    for (unsigned int i = 0; i < shape.size(); ++i) {
      EXPECT_EQ(shape[i], static_cast<unsigned int>(expected_shape[i])) << "Shapes differ at index " << i ;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return res;
}
