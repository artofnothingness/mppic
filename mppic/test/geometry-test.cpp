#ifdef DO_BENCHMARKS
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#endif

#include <catch2/catch.hpp>

#include <cmath>
#include <vector>
#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>

#include "utils/geometry.hpp"

static auto toStr = [](auto && container) {
    std::stringstream ss;
    ss << "[ ";
    for (auto & val : container) {
      ss << val << ' ';
    }
    ss << "]";

    return ss.str();
  };

TEST_CASE("Closest points on Line Segments 2D", "[geometry]") {
  using T = float;
  using Array = xt::xarray<T>;

  // clang-format off
  Array points = {{3.0f, 3.0f},
    {-3.0f, -3.0f}};

  Array line_points = {{{0.0f, 0.0f},
    {1.0f, 1.0f},
    {2.0f, 2.0f},
    {4.0f, 2.0f}},

    {{0.0f, 0.0f},
      {-1.0f, -1.0f},
      {-2.0f, -2.0f},
      {-4.0f, -2.0f}}};
  Array result;
  // clang-format on

  REQUIRE_NOTHROW(
    result = mppi::geometry::closestPointsOnLinesSegment2D(
      points, line_points));

  size_t bench_i = GENERATE(10, 100);
  auto bench_points = Array::from_shape({bench_i, 2});
  auto bench_lines = Array::from_shape({300, 20, 2});
  bench_lines.fill(20);


#ifdef DO_BENCHMARKS
  WARN(
    "Points in bench: [ " << bench_i << " ]" << '\n' <<
      "Lines shape: " << toStr(bench_lines.shape()));
  BENCHMARK("Closest Points on Segments Benchmark") {
    return mppi::geometry::closestPointsOnLinesSegment2D(
      bench_points,
      bench_lines);
  };
#endif

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

    // Middle of the line check
    CHECK(
      result(0, 2, 0, 0) ==
      (line_points(0, 3, 0) - line_points(0, 2, 0)) / 2 +
      line_points(0, 2, 0));

    CHECK(result(0, 2, 0, 1) == line_points(0, 3, 1));
    CHECK(result(0, 2, 0, 1) == line_points(0, 2, 1));
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

TEST_CASE("Distance Points To Line Segments 2D", "[geometry]") {
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

  REQUIRE_NOTHROW(
    result = mppi::geometry::distPointsToLineSegments2D(points, line_points));

  size_t bench_i = GENERATE(10, 30, 100);
  auto bench_points = Array::from_shape({bench_i, 2});
  auto bench_lines = Array::from_shape({300, 20, 2});

#ifdef DO_BENCHMARKS
  WARN(
    "Points in benchmark: [ " << bench_i << " ]" << '\n' <<
      "Lines shape: " << toStr(bench_lines.shape()));
  BENCHMARK("Distance from Lines to Points Benchmark") {
    return mppi::geometry::distPointsToLineSegments2D(
      bench_points,
      bench_lines);
  };
#endif

  SECTION("Check Results") {
    INFO("Points: \n" << points);
    INFO("Line points: \n" << line_points);
    INFO("Result: \n" << result);

    CHECK(result(0, 0, 0) == Approx(sqrt(8)));
    CHECK(result(0, 1, 0) == Approx(sqrt(2)));
  }

  SECTION("Shape Check") {
    INFO("Points shape: \n" << toStr(points.shape()));
    INFO("Line points shape: \n" << toStr(line_points.shape()));
    INFO("Result shape: \n" << toStr(result.shape()));

    auto result_1_dim = line_points.shape()[0];
    auto result_2_dim = line_points.shape()[1] - 1;
    auto result_3_dim = points.shape()[0];

    CHECK(result.shape()[0] == result_1_dim);
    CHECK(result.shape()[1] == result_2_dim);
    CHECK(result.shape()[2] == result_3_dim);
  }
}
