// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#include <catch2/catch_all.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_core/goal_checker.hpp>

#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>

#include "mppic/optimizers/xtensor/optimizer.hpp"
#include "mppic/optimizers/xtensor/motion_models.hpp"

#include "mppic/parameters_handler.hpp"

#include "utils/utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;


TEST_CASE("Optimizer doesn't fail")
{
  SECTION("evalControl with different critics") {
    // Settings
    int batch_size = 400;
    int time_steps = 15;
    unsigned int path_points = 50u;
    int iteration_count = 1;
    double lookahead_distance = 10.0;
    std::vector<std::string> critics;

    bool consider_footprint = GENERATE(true, false);
    std::string motion_model = GENERATE("DiffDrive", "Omni");

    TestCostmapSettings costmap_settings{};
    auto costmap_ros = getDummyCostmapRos(costmap_settings);
    auto costmap = costmap_ros->getCostmap();

    TestPose start_pose = costmap_settings.getCenterPose();
    double path_step = costmap_settings.resolution;

    TestPathSettings path_settings{start_pose, path_points, path_step, path_step};
    TestOptimizerSettings optimizer_settings{batch_size, time_steps, iteration_count,
      lookahead_distance, motion_model, consider_footprint};

    SECTION("Obstacles free critics test/benchmark") {
      std::cout << "\nObstacles free critics test/benchmark" << "\n";
      critics = GENERATE(
        std::vector<std::string>{},
        std::vector<std::string>{{"GoalCritic"}},
        std::vector<std::string>{{"GoalAngleCritic"}},
        std::vector<std::string>{{"LocalGoalCritic"}},
        std::vector<std::string>{{"PreferForwardCritic"}},
        std::vector<std::string>{{"ReferenceTrajectoryCritic"}},
        std::vector<std::string>{{"PathAngleCritic"}},
        std::vector<std::string>{{"TwirlingCritic"}}
      );
    }

    SECTION("Obstacles dependent critics test/benchmark") {
      {
        std::cout << "\nObstacles dependent critics test/benchmark" << "\n";
        critics = GENERATE(
          std::vector<std::string>{{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
            {"ReferenceTrajectoryCritic"}, {"LocalGoalCritic"}},
          std::vector<std::string>{{"ObstaclesCritic"}}
        );

        unsigned int offset = 4;
        unsigned int obstacle_size = offset * 2;

        unsigned char obstacle_cost = 250;

        auto [obst_x, obst_y] = costmap_settings.getCenterIJ();

        obst_x = obst_x - offset;
        obst_y = obst_y - offset;
        addObstacle(costmap, {obst_x, obst_y, obstacle_size, obstacle_cost});
      }
    }

    printInfo(optimizer_settings, path_settings, critics);
    auto node = getDummyNode(optimizer_settings, critics);
    auto parameters_handler = std::make_unique<mppi::ParametersHandler>(node);
    auto optimizer = getDummyOptimizer(node, costmap_ros, parameters_handler.get());

    // evalControl args
    auto pose = getDummyPointStamped(node, start_pose);
    auto velocity = getDummyTwist();
    auto path = getIncrementalDummyPath(node, path_settings);
    nav2_core::GoalChecker * dummy_goal_checker{nullptr};

#ifdef DO_BENCHMARKS
    BENCHMARK("evalControl Benchmark") {
      return optimizer.evalControl(pose, velocity, path, dummy_goal_checker);
    };
#else
    REQUIRE_NOTHROW(optimizer.evalControl(pose, velocity, path, dummy_goal_checker));
#endif

  }
}


TEST_CASE("Ackermann Throw Exception on predict")
{
  mppi::xtensor::AckermannMotionModel model;
  xt::xtensor<double, 2> in;
  mppi::xtensor::models::StateIdxes idx;
  REQUIRE(!model.isHolonomic());
  REQUIRE_THROWS_AS(model.predict(in, idx), std::runtime_error);
}
