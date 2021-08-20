#define CATCH_CONFIG_RUNNER
#define CATCH_CONFIG_ENABLE_BENCHMARKING

#include <catch2/catch.hpp>

#include <vector>
#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>

#include "utils/geometry.hpp"

static auto toStr = [](auto &&container) {
  std::stringstream ss;
  ss << "[ ";
  for (auto val : container) {
    ss << val << ' ';
  }
  ss << "]";

  return ss.str();
};

TEST_CASE("PointTensor 2D, LineTensor 3D", "[closestPointToLineSegment2D]") {
  using T = float;
  using Array = xt::xarray<T>;

  // clang-format off
  Array points = {{3.0f, 3.0f}, 
                  {-3.0f, -3.0f}};

  Array line_points = {{{0.0f, 0.0f}, 
                        {1.0f, 1.0f}, 
                        {2.0f, 2.0f}}, 

                       {{0.0f, 0.0f}, 
                        {-1.0f, -1.0f}, 
                        {-2.0f, -2.0f}}};
  Array result;
  // clang-format on

  REQUIRE_NOTHROW(result = mppi::geometry::closestPointsOnLinesSegment2D(
                      points, line_points));

  SECTION("Check Results") {

    INFO("Points: \n" << points);
    INFO("Line points: \n" << line_points);
    INFO("Result: \n" << result);

    CHECK(result(0, 0, 0, 0) == line_points(0, 1, 0));
    CHECK(result(0, 0, 0, 1) == line_points(0, 1, 1));

    CHECK(result(0, 0, 1, 0) == line_points(0, 0, 0));
    CHECK(result(0, 0, 1, 1) == line_points(0, 0, 1));

    CHECK(result(1, 0, 1, 0) == line_points(1, 1, 0));
    CHECK(result(1, 0, 1, 1) == line_points(1, 1, 1));

    CHECK(result(1, 1, 1, 0) == line_points(1, 2, 0));
    CHECK(result(1, 1, 1, 1) == line_points(1, 2, 1));

    CHECK(result(1, 0, 0, 0) == line_points(1, 0, 0));
    CHECK(result(1, 0, 0, 1) == line_points(1, 0, 1));

    CHECK(result(1, 1, 0, 0) == line_points(1, 1, 0));
    CHECK(result(1, 1, 0, 1) == line_points(1, 1, 1));
  }

  SECTION("Shape Check") {

    INFO("Points shape: \n" << toStr(points.shape()));
    INFO("Line points shape: \n" << toStr(line_points.shape()));
    INFO("Result shape: \n" << toStr(result.shape()));

    auto result_1_dim = line_points.shape()[0];
    auto result_2_dim = line_points.shape()[1] - 1;
    auto result_3_dim = points.shape()[0];
    auto result_4_dim = points.shape()[1];

    CHECK(result.shape()[0] == result_1_dim);
    CHECK(result.shape()[1] == result_2_dim);
    CHECK(result.shape()[2] == result_3_dim);
    CHECK(result.shape()[3] == result_4_dim);
  }
}

int main(int argc, char *argv[]) {
  int result = Catch::Session().run(argc, argv);
  return result;
}
