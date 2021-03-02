#define BOOST_TEST_DYN_LINK

#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/util/math/distance.h"

using namespace ai_server;

BOOST_AUTO_TEST_SUITE(math)

BOOST_AUTO_TEST_CASE(distance, *boost::unit_test::tolerance(0.0000001)) {
  using util::math::distance;
  using vec_xy = std::tuple<double, double>;

  BOOST_TEST(distance(vec_xy{-1, -1}, vec_xy{1, 1}) == std::hypot(2, 2));
  BOOST_TEST(distance(vec_xy{1, -1}, vec_xy{-1, 1}) == std::hypot(2, 2));
}

BOOST_AUTO_TEST_CASE(distance3d, *boost::unit_test::tolerance(0.0000001)) {
  using util::math::distance3d;
  using vec_xyz = std::tuple<double, double, double>;

  BOOST_TEST(distance3d(vec_xyz{-1, 0, -1}, vec_xyz{1, 0, 1}) == std::hypot(2, 2));
  BOOST_TEST(distance3d(vec_xyz{0, -1, -1}, vec_xyz{0, 1, 1}) == std::hypot(2, 2));
  BOOST_TEST(distance3d(vec_xyz{1, 1, -1}, vec_xyz{-1, -1, 1}) == std::hypot(2, 2, 2));
  BOOST_TEST(distance3d(vec_xyz{-1, 1, -1}, vec_xyz{1, -1, 1}) == std::hypot(2, 2, 2));
  BOOST_TEST(distance3d(vec_xyz{-1, -1, -1}, vec_xyz{1, 1, 1}) == std::hypot(2, 2, 2));
  BOOST_TEST(distance3d(vec_xyz{1, -1, -1}, vec_xyz{-1, 1, 1}) == std::hypot(2, 2, 2));
}

BOOST_AUTO_TEST_SUITE_END()
