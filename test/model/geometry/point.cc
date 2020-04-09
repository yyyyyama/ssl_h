#define BOOST_TEST_DYN_LINK

#include <array>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/geometry/point.h"

using point = ai_server::model::geometry::point<double>;

// テスト用の構造体
struct velocity {
  double vx;
  double vy;

  velocity(double x, double y) : vx(x), vy(y) {}
};

BOOST_AUTO_TEST_SUITE(model_geometry_point)

// テスト用構造体のチェック
BOOST_AUTO_TEST_CASE(check_struct_velocity) {
  const velocity p{2.0, 7.0};

  BOOST_TEST(p.vx == 2.0);
  BOOST_TEST(p.vy == 7.0);
}

// T as()
BOOST_AUTO_TEST_CASE(test_as) {
  const point p{2.0, 7.0};
  const auto result = p.as<velocity>();

  BOOST_TEST(result.vx == p.x);
  BOOST_TEST(result.vy == p.y);
}

// std::pair<T> as_pair()
BOOST_AUTO_TEST_CASE(test_as_pair) {
  const point p{2.0, 7.0};
  const auto result = p.as_pair();

  BOOST_TEST(result.first == p.x);
  BOOST_TEST(result.second == p.y);
}

// Eigen::Matrix<T, 2, 1> as_vector();
BOOST_AUTO_TEST_CASE(test_as_vector) {
  const point p{2.0, 7.0};
  const auto result = p.as_vector();

  BOOST_TEST(result.x() == p.x);
  BOOST_TEST(result.y() == p.y);
}

BOOST_AUTO_TEST_SUITE_END()
