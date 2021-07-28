#include <gtest/gtest.h>

#include "mppi/Optimizer.hpp" 

using namespace ultra::mppi::optimization;

/// @brief Test Fixture
class OptimizerTest : public ::testing::Test {
protected:

  void SetUp() override {
    opt.model_dt = 0.1;
    opt.time_steps = 20;
    opt.batch_size = 100;
    opt.v_std = 0.1;
    opt.w_std = 0.1;
    opt.v_limit = 0.5;
    opt.w_limit = 1.0;
    opt.n_optimizations = 2;
    opt.lookahead_dist = 1.2;

    opt.reset();

    auto shape = opt.batches.shape();
    std::vector<int> expected_shape = {
      opt.batch_size, 
      opt.time_steps, 
      opt.control_dim
    };

    for (int i =0 ; i < shape.size(); ++i) {
      EXPECT_EQ(, expected_batches_size);
    }
  }

  void TearDown() override {}

  Optimizer<float> opt;
};

TEST_F(OptimizerTest, Test_one) {
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
