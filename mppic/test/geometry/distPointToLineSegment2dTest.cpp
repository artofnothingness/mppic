#define CATCH_CONFIG_RUNNER
#define CATCH_CONFIG_ENABLE_BENCHMARKING

#include <catch2/catch.hpp>

#include <vector>
#include <xtensor/xarray.hpp>

#include "utils/geometry.hpp"

TEST_CASE("One Line Check", "[distPointToLineSegment2D]") {

  using T = float;
  using Array = xt::xarray<T>;

  SECTION("End of the line must be taken") {
    Array start = {0, 0};
    Array end = {1, 0};
    Array pt = {2, 2};
    auto result = mppi::geometry::distPointToLineSegment2D(pt, start, end);

    CHECK(result(0) == Approx(1.0f));
    CHECK(result(1) == Approx(0.0f));
  }

  /* SECTION("Middle of the line must be taken") { */
  /*   Array start = {0, 0}; */
  /*   Array end = {1, 0}; */
  /*   Array pt = {0.5, 1.0}; */
  /*   auto result = mppi::geometry::distPointToLineSegment2D(pt, start, end); */

  /*   CHECK(result(0) == Approx(0.5f)); */
  /*   CHECK(result(1) == Approx(0.0f)); */
  /* } */

  /* SECTION("Begin of the line must be taken") { */
  /*   Array start = {0, 0}; */
  /*   Array end = {1, 0}; */
  /*   Array pt = {-1, -1}; */
  /*   auto result = mppi::geometry::distPointToLineSegment2D(pt, start, end); */

  /*   CHECK(result(0) == Approx(0.0f)); */
  /*   CHECK(result(0) == Approx(0.0f)); */
  /* } */
}

/* TEST_CASE("Multi line Check", "[closestPointToLineSegment2D]") { */

/*   using T = float; */
/*   using Array = xt::xarray<T>; */

/*   SECTION("End of the line must be taken") { */
/*     T pt_x = GENERATE(3, 5, 10); */
/*     T pt_y = GENERATE(0, 1, 2); */
/*     Array pt = {pt_x, pt_y}; */

/*     Array start = {{0, 0}, {1, 1}}; */
/*     Array end = {{1, 0}, {2, 1}}; */
/*     auto result = mppi::geometry::distPointToLineSegment2D(pt, start, end); */

/*     INFO("Point x: " << pt_x); */
/*     INFO("Point y: " << pt_y); */

/*     CHECK(result(0, 0) == Approx(end(0, 0))); */
/*     CHECK(result(0, 1) == Approx(end(0, 1))); */

/*     CHECK(result(1, 0) == Approx(end(1, 0))); */
/*     CHECK(result(1, 1) == Approx(end(1, 1))); */
/*   } */

/*   SECTION("Begin of the line must be taken") { */
/*     T pt_x = GENERATE(-3, -5, -10); */
/*     T pt_y = GENERATE(0, -1, -2); */
/*     Array pt = {pt_x, pt_y}; */

/*     Array start = {{0, 0}, {1, 1}}; */
/*     Array end = {{1, 0}, {2, 1}}; */
/*     auto result = mppi::geometry::distPointToLineSegment2D(pt, start, end); */

/*     INFO("Point x: " << pt_x); */
/*     INFO("Point y: " << pt_y); */

/*     CHECK(result(0, 0) == Approx(start(0, 0))); */
/*     CHECK(result(0, 1) == Approx(start(0, 1))); */

/*     CHECK(result(1, 0) == Approx(start(1, 0))); */
/*     CHECK(result(1, 1) == Approx(start(1, 1))); */
/*   } */

/*   SECTION("Middle of the line must be taken") { */
/*     T pt_x = 5; */
/*     T pt_y = 10; */

/*     Array pt = {pt_x, pt_y}; */

/*     Array start = {{0, 0}, {0, 5}}; */

/*     Array end = {{10, 0}, {10, 5}}; */

/*     auto result = mppi::geometry::distPointToLineSegment2D(pt, start, end); */

/*     CHECK(result(0, 0) == */
/*           Approx((end(0, 0) - start(0, 0)) / 2.0f + start(0, 0))); */

/*     CHECK(result(0, 1) == Approx(end(0, 1))); */

/*     CHECK(result(1, 0) == */
/*           Approx((end(1, 0) - start(1, 0)) / 2.0f + start(1, 0))); */
/*     CHECK(result(1, 1) == Approx(end(1, 1))); */
/*   } */

/*   using T = float; */
/*   using Array = xt::xarray<T>; */

/*   SECTION("End of the line must be taken with multiple points") { */
/*     T pt_x = 3; */
/*     T pt_y = 0; */
/*     Array pt = {{pt_x, pt_y}, {pt_x, pt_y}}; */

/*     Array start = {{0, 0}, {1, 1}}; */
/*     Array end = {{1, 0}, {2, 1}}; */
/*     auto result = mppi::geometry::distPointToLineSegment2D(pt, start, end); */

/*     INFO("Point x: " << pt_x); */
/*     INFO("Point y: " << pt_y); */

/*     CHECK(result(0, 0, 0) == Approx(end(0, 0))); */
/*     CHECK(result(0, 0, 1) == Approx(end(0, 1))); */

/*     CHECK(result(0, 1, 0) == Approx(end(1, 0))); */
/*     CHECK(result(0, 1, 1) == Approx(end(1, 1))); */

/*     CHECK(result(1, 0, 0) == Approx(end(0, 0))); */
/*     CHECK(result(1, 0, 1) == Approx(end(0, 1))); */

/*     CHECK(result(1, 1, 0) == Approx(end(1, 0))); */
/*     CHECK(result(1, 1, 1) == Approx(end(1, 1))); */
/*   } */

/*   SECTION("End of the line must be taken with batch of points ") { */
/*     T pt_x = 3; */
/*     T pt_y = 0; */
/*     Array pt = {{pt_x, pt_y}, {pt_x, pt_y}}; */

/*     Array start = {{{0, 0}, {1, 1}}, {{0, 0}, {1, 1}}}; */
/*     Array end = {{{1, 0}, {2, 1}}, {{1, 0}, {2, 1}}}; */
/*     auto result = mppi::geometry::distPointToLineSegment2D(pt, start, end); */

/*     INFO("Point x: " << pt_x); */
/*     INFO("Point y: " << pt_y); */

/*     CHECK (result.dimension() == 4); */

/*     CHECK(result(0, 0, 0, 0) == Approx(end(0, 0))); */
/*     CHECK(result(0, 0, 0, 1) == Approx(end(0, 1))); */

/*     CHECK(result(0, 1, 1, 0) == Approx(end(1, 0))); */
/*     CHECK(result(0, 1, 1, 1) == Approx(end(1, 1))); */

/*     CHECK(result(1, 0, 0, 0) == Approx(end(0, 0))); */
/*     CHECK(result(1, 0, 0, 1) == Approx(end(0, 1))); */

/*     CHECK(result(1, 1, 1, 0) == Approx(end(1, 0))); */
/*     CHECK(result(1, 1, 1, 1) == Approx(end(1, 1))); */
/*   } */
/* } */

int main(int argc, char *argv[]) {
  int result = Catch::Session().run(argc, argv);
  return result;
}
