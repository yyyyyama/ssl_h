#define BOOST_TEST_DYN_LINK

#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>
#include <Eigen/Dense>

#include "ai_server/util/math/distance.h"

using namespace ai_server;

// テストで使う構造体
struct pos_xy {
  double x;
  double y;
};
struct pos_xyz {
  double x;
  double y;
  double z;
};

BOOST_AUTO_TEST_SUITE(math)

BOOST_AUTO_TEST_CASE(distance, *boost::unit_test::tolerance(0.0000001)) {
  using util::math::distance;
  using vec_xy = Eigen::Vector2d;

  // pos, pos
  BOOST_TEST(distance(pos_xy{1, 0}, pos_xy{-1, 0}) == 2);
  BOOST_TEST(distance(pos_xy{0, 1}, pos_xy{0, -1}) == 2);

  // pos, vector
  BOOST_TEST(distance(pos_xy{-1, 0}, vec_xy{1, 0}) == 2);
  BOOST_TEST(distance(pos_xy{0, -1}, vec_xy{0, 1}) == 2);

  // vector, pos
  BOOST_TEST(distance(vec_xy{1, 1}, pos_xy{-1, -1}) == std::hypot(2, 2));
  BOOST_TEST(distance(vec_xy{-1, 1}, pos_xy{1, -1}) == std::hypot(2, 2));

  // vector, vector
  BOOST_TEST(distance(vec_xy{-1, -1}, vec_xy{1, 1}) == std::hypot(2, 2));
  BOOST_TEST(distance(vec_xy{1, -1}, vec_xy{-1, 1}) == std::hypot(2, 2));
}

BOOST_AUTO_TEST_CASE(distance3d, *boost::unit_test::tolerance(0.0000001)) {
  using util::math::distance3d;
  using vec_xyz = Eigen::Vector3d;

  // pos, pos
  BOOST_TEST(distance3d(pos_xyz{0, 0, 1}, pos_xyz{0, 0, -1}) == 2);
  BOOST_TEST(distance3d(pos_xyz{0, 0, -1}, pos_xyz{0, 0, 1}) == 2);

  BOOST_TEST(distance3d(pos_xyz{1, 0, 0}, pos_xyz{-1, 0, 0}) == 2);
  BOOST_TEST(distance3d(pos_xyz{0, 1, 0}, pos_xyz{0, -1, 0}) == 2);
  BOOST_TEST(distance3d(pos_xyz{-1, 0, 0}, pos_xyz{1, 0, 0}) == 2);
  BOOST_TEST(distance3d(pos_xyz{0, -1, 0}, pos_xyz{0, 1, 0}) == 2);
  BOOST_TEST(distance3d(pos_xyz{1, 1, 0}, pos_xyz{-1, -1, 0}) == std::hypot(2, 2));
  BOOST_TEST(distance3d(pos_xyz{-1, 1, 0}, pos_xyz{1, -1, 0}) == std::hypot(2, 2));

  // pos, vector
  BOOST_TEST(distance3d(pos_xyz{-1, -1, 0}, vec_xyz{1, 1, 0}) == std::hypot(2, 2));
  BOOST_TEST(distance3d(pos_xyz{1, -1, 0}, vec_xyz{-1, 1, 0}) == std::hypot(2, 2));

  BOOST_TEST(distance3d(pos_xyz{1, 0, 1}, vec_xyz{-1, 0, -1}) == std::hypot(2, 2));
  BOOST_TEST(distance3d(pos_xyz{0, 1, 1}, vec_xyz{0, -1, -1}) == std::hypot(2, 2));
  BOOST_TEST(distance3d(pos_xyz{-1, 0, 1}, vec_xyz{1, 0, -1}) == std::hypot(2, 2));
  BOOST_TEST(distance3d(pos_xyz{0, -1, 1}, vec_xyz{0, 1, -1}) == std::hypot(2, 2));

  // vector, pos
  BOOST_TEST(distance3d(vec_xyz{1, 1, 1}, pos_xyz{-1, -1, -1}) == std::hypot(2, 2, 2));
  BOOST_TEST(distance3d(vec_xyz{-1, 1, 1}, pos_xyz{1, -1, -1}) == std::hypot(2, 2, 2));
  BOOST_TEST(distance3d(vec_xyz{-1, -1, 1}, pos_xyz{1, 1, -1}) == std::hypot(2, 2, 2));
  BOOST_TEST(distance3d(vec_xyz{1, -1, 1}, pos_xyz{-1, 1, -1}) == std::hypot(2, 2, 2));

  BOOST_TEST(distance3d(vec_xyz{1, 0, -1}, pos_xyz{-1, 0, 1}) == std::hypot(2, 2));
  BOOST_TEST(distance3d(vec_xyz{0, 1, -1}, pos_xyz{0, -1, 1}) == std::hypot(2, 2));

  // vector, vector
  BOOST_TEST(distance3d(vec_xyz{-1, 0, -1}, vec_xyz{1, 0, 1}) == std::hypot(2, 2));
  BOOST_TEST(distance3d(vec_xyz{0, -1, -1}, vec_xyz{0, 1, 1}) == std::hypot(2, 2));
  BOOST_TEST(distance3d(vec_xyz{1, 1, -1}, vec_xyz{-1, -1, 1}) == std::hypot(2, 2, 2));
  BOOST_TEST(distance3d(vec_xyz{-1, 1, -1}, vec_xyz{1, -1, 1}) == std::hypot(2, 2, 2));
  BOOST_TEST(distance3d(vec_xyz{-1, -1, -1}, vec_xyz{1, 1, 1}) == std::hypot(2, 2, 2));
  BOOST_TEST(distance3d(vec_xyz{1, -1, -1}, vec_xyz{-1, 1, 1}) == std::hypot(2, 2, 2));
}

BOOST_AUTO_TEST_SUITE_END()
