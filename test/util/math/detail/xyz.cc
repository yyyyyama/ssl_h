#define BOOST_TEST_DYN_LINK

#include <boost/test/data/test_case.hpp>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

#include "ai_server/util/math/distance.h"

BOOST_AUTO_TEST_SUITE(math)

// テストで使う構造体
struct point_x {
  double x;
};
struct point_xy {
  double x;
  double y;
};
struct point_xyz {
  double x;
  double y;
  double z;
};

namespace math_dt = ai_server::util::math::detail;

BOOST_DATA_TEST_CASE(xyz_test,
                     boost::unit_test::data::make({68547.76, -56.4, 3.3467, -47567.3}) ^
                         boost::unit_test::data::make({-47.363, 25477.53, 73135.467,
                                                       -0.258346}) ^
                         boost::unit_test::data::make({73.765, -3772.54, -6.5257, -661.8}),
                     x, y, z) {
  { // x

    // 構造体
    BOOST_TEST(math_dt::x(point_x{x}) == x);
    BOOST_TEST(math_dt::x(point_xy{x, y}) == x);
    BOOST_TEST(math_dt::x(point_xyz{x, y, z}) == x);
    // クラス
    BOOST_TEST(math_dt::x(Eigen::Matrix<double, 1, 1>{x}) == x);
    BOOST_TEST(math_dt::x(Eigen::Vector2d{x, y}) == x);
    BOOST_TEST(math_dt::x(Eigen::Vector3d{x, y, z}) == x);
    // tuple
    BOOST_TEST(math_dt::x(std::make_tuple(x)) == x);
    BOOST_TEST(math_dt::x(std::make_tuple(x, y)) == x);
    BOOST_TEST(math_dt::x(std::make_tuple(x, y, z)) == x);
  }

  { // y

    // 構造体
    BOOST_TEST(math_dt::y(point_xy{x, y}) == y);
    BOOST_TEST(math_dt::y(point_xyz{x, y, z}) == y);
    // クラス
    BOOST_TEST(math_dt::y(Eigen::Vector2d{x, y}) == y);
    BOOST_TEST(math_dt::y(Eigen::Vector3d{x, y, z}) == y);
    // tuple
    BOOST_TEST(math_dt::y(std::make_tuple(x, y)) == y);
    BOOST_TEST(math_dt::y(std::make_tuple(x, y, z)) == y);
  }

  { // z

    // 構造体
    BOOST_TEST(math_dt::z(point_xyz{x, y, z}) == z);
    // クラス
    BOOST_TEST(math_dt::z(Eigen::Vector3d{x, y, z}) == z);
    // tuple
    BOOST_TEST(math_dt::z(std::make_tuple(x, y, z)) == z);
  }
}

BOOST_AUTO_TEST_SUITE_END()
