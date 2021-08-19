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

TEST_CASE("Point 1D, Line 1D", "[closestPointToLineSegment2D]") {

  using T = float;
  using Array = xt::xarray<T>;

  SECTION("Begin of the line must be taken") {
    Array start = {0, 0};
    Array end = {1, 0};
    Array pt = {-1, -1};
    auto result = mppi::geometry::closestPointToLineSegment2D(pt, start, end);

    INFO("Points: \n" << pt);
    INFO("Start: \n" << start);
    INFO("End: \n" << end);
    INFO("Result: \n" << result);

    CHECK(result.dimension() == 1);
    CHECK(result(0) == Approx(0.0f));
    CHECK(result(0) == Approx(0.0f));
  }

  SECTION("End of the line must be taken") {
    Array start = {0, 0};
    Array end = {1, 0};
    Array pt = {2, 2};
    auto result = mppi::geometry::closestPointToLineSegment2D(pt, start, end);

    INFO("Points: \n" << pt);
    INFO("Start: \n" << start);
    INFO("End: \n" << end);
    INFO("Result: \n" << result);

    CHECK(result.dimension() == 1);
    CHECK(result(0) == Approx(1.0f));
    CHECK(result(1) == Approx(0.0f));
  }

  SECTION("Middle of the line must be taken") {
    Array start = {0, 0};
    Array end = {1, 0};
    Array pt = {0.5, 1.0};
    auto result = mppi::geometry::closestPointToLineSegment2D(pt, start, end);

    INFO("Points: \n" << pt);
    INFO("Start: \n" << start);
    INFO("End: \n" << end);
    INFO("Result: \n" << result);

    CHECK(result.dimension() == 1);
    CHECK(result(0) == Approx(0.5f));
    CHECK(result(1) == Approx(0.0f));
  }
}

TEST_CASE("Point 1D, Line 2D", "[closestPointToLineSegment2D]") {

  using T = float;
  using Array = xt::xarray<T>;

  SECTION("Begin of the line must be taken") {
    T pt_x = GENERATE(-3, -5, -10);
    T pt_y = GENERATE(0, -1, -2);
    Array pt = {pt_x, pt_y};

    Array start = {{0, 0}, {1, 1}};
    Array end = {{1, 0}, {2, 1}};

    auto result = mppi::geometry::closestPointToLineSegment2D(pt, start, end);

    INFO("Points: \n" << pt);
    INFO("Start: \n" << start);
    INFO("End: \n" << end);
    INFO("Result: \n" << result);

    CHECK(result.dimension() == 2); // line_dims.. x pt_dims..
    CHECK(result(0) == start(0));
    CHECK(result(1) == start(1));
  }

  SECTION("Middle of the line must be taken") {
    T pt_x = 5;
    T pt_y = 10;

    Array pt = {pt_x, pt_y};
    Array start = {{0, 0}, {0, 5}};
    Array end = {{10, 0}, {10, 5}};

    auto result = mppi::geometry::closestPointToLineSegment2D(pt, start, end);

    INFO("Points: \n" << pt);
    INFO("Start: \n" << start);
    INFO("End: \n" << end);
    INFO("Result: \n" << result);
    CHECK(result.dimension() == 2); // line_dims.. x pt_dims..

    CHECK(result(0, 0) ==
          Approx((end(0, 0) - start(0, 0)) / 2.0f + start(0, 0)));
    CHECK(result(0, 1) == Approx(end(0, 1)));
    CHECK(result(1, 0) ==
          Approx((end(1, 0) - start(1, 0)) / 2.0f + start(1, 0)));
    CHECK(result(1, 1) == Approx(end(1, 1)));
  }

  SECTION("End of the line must be taken") {
    T pt_x = GENERATE(3, 5, 10);
    T pt_y = GENERATE(0, 1, 2);
    Array pt = {pt_x, pt_y};

    Array start = {{0, 0}, {1, 1}};
    Array end = {{1, 0}, {2, 1}};
    auto result = mppi::geometry::closestPointToLineSegment2D(pt, start, end);

    INFO("Points: \n" << pt);
    INFO("Start: \n" << start);
    INFO("End: \n" << end);
    INFO("Result: \n" << result);
    CHECK(result.dimension() == 2); // line_dims.. x pt_dims..

    CHECK(result(0) == end(0));
    CHECK(result(1) == end(1));
  }
}

TEST_CASE("Point 2D, Line 3D", "[closestPointToLineSegment2D]") {
  using T = float;
  using Array = xt::xarray<T>;

  T pt_x = 3;
  T pt_y = 0;

  Array pt = {{pt_x, pt_y}, {pt_x, pt_y}};
  Array start = {{{0, 0}, {1, 1}}, {{0, 0}, {1, 1}}};
  Array end = {{{1, 0}, {2, 1}}, {{1, 0}, {2, 1}}};
  Array result;

  REQUIRE_NOTHROW(
      result = mppi::geometry::closestPointToLineSegment2D(pt, start, end));

  SECTION("Shape Check") {

    INFO("Points shape: \n" << toStr(pt.shape()));
    INFO("Start shape: \n" << toStr(start.shape()));
    INFO("End shape: \n" << toStr(end.shape()));
    INFO("Result shape: \n" << toStr(result.shape()));

    auto result_1_dim = start.shape()[0];
    auto result_2_dim = start.shape()[1];
    auto result_3_dim = pt.shape()[0];
    auto result_4_dim = pt.shape()[1];

    CHECK(result.shape()[0] == result_1_dim);
    CHECK(result.shape()[1] == result_2_dim);
    CHECK(result.shape()[2] == result_3_dim);
    CHECK(result.shape()[3] == result_4_dim);
  }

  SECTION("End of the line must be taken") {

    INFO("Points: \n" << pt);
    INFO("Start: \n" << start);
    INFO("End: \n" << end);
    INFO("Result: \n" << result);

    CHECK(result(0, 0, 0) == end(0, 0));
    CHECK(result(0, 1, 0) == end(0, 1));
    CHECK(result(1, 0, 1) == end(1, 0));
    CHECK(result(1, 1, 1) == end(1, 1));
  }
}

int main(int argc, char *argv[]) {
  int result = Catch::Session().run(argc, argv);
  return result;
}
