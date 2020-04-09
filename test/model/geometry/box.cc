#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai_server/model/geometry/box.h"

BOOST_AUTO_TEST_SUITE(model_geometry_box)

// テスト用の構造体
template <class T>
struct area {
  T min_position;
  T max_position;

  area(const T& min, const T& max) : min_position(min), max_position(max) {}
};

template <class T, class U>
struct posion_pair {
  T position1;
  U position2;

  posion_pair(const T& p1, const U& p2) : position1(p1), position2(p2) {}
};

using box   = ai_server::model::geometry::box<double>;
using point = ai_server::model::geometry::point<double>;

// テスト用構造体のチェック
BOOST_AUTO_TEST_CASE(check_struct_area) {
  const area<Eigen::Vector2d> a{{2.0, 7.0}, {11.0, 13.0}};

  BOOST_TEST(a.min_position.x() == 2.0);
  BOOST_TEST(a.min_position.y() == 7.0);
  BOOST_TEST(a.max_position.x() == 11.0);
  BOOST_TEST(a.max_position.y() == 13.0);
}

// テスト用構造体のチェック
BOOST_AUTO_TEST_CASE(check_struct_position_pair) {
  const posion_pair<Eigen::Vector2d, Eigen::Vector2d> p{{2.0, 7.0}, {11.0, 13.0}};

  BOOST_TEST(p.position1.x() == 2.0);
  BOOST_TEST(p.position1.y() == 7.0);
  BOOST_TEST(p.position2.x() == 11.0);
  BOOST_TEST(p.position2.y() == 13.0);
}

// Box<Point> as();
BOOST_AUTO_TEST_CASE(test_as1) {
  const box b{point{2.0, 7.0}, point{11.0, 13.0}};
  const auto result = b.as<area, Eigen::Vector2d>();

  BOOST_TEST(result.min_position.x() == b.min.x);
  BOOST_TEST(result.min_position.y() == b.min.y);
  BOOST_TEST(result.max_position.x() == b.max.x);
  BOOST_TEST(result.max_position.y() == b.max.y);
}

// Box<Point, Point> as();
BOOST_AUTO_TEST_CASE(test_as2) {
  const box b{point{2.0, 7.0}, point{11.0, 13.0}};
  const auto result = b.as<posion_pair, Eigen::Vector2d>();

  BOOST_TEST(result.position1.x() == b.min.x);
  BOOST_TEST(result.position1.y() == b.min.y);
  BOOST_TEST(result.position2.x() == b.max.x);
  BOOST_TEST(result.position2.y() == b.max.y);
}

// std::pair<Point> as_pair();
BOOST_AUTO_TEST_CASE(test_as_pair) {
  const box b{point{2.0, 7.0}, point{11.0, 13.0}};
  const auto result = b.as_pair();

  BOOST_TEST(result.first.x() == b.min.x);
  BOOST_TEST(result.first.y() == b.min.y);
  BOOST_TEST(result.second.x() == b.max.x);
  BOOST_TEST(result.second.y() == b.max.y);
}

// boost::geometry::model::box<Point> as_box();
BOOST_AUTO_TEST_CASE(test_as_box) {
  const box b{point{2.0, 7.0}, point{11.0, 13.0}};
  const auto result = b.as_box();

  BOOST_TEST(result.min_corner().x() == b.min.x);
  BOOST_TEST(result.min_corner().y() == b.min.y);
  BOOST_TEST(result.max_corner().x() == b.max.x);
  BOOST_TEST(result.max_corner().y() == b.max.y);
}

BOOST_AUTO_TEST_SUITE_END()
