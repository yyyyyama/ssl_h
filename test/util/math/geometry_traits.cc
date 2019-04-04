#define BOOST_TEST_DYN_LINK

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/test/unit_test.hpp>
#include <Eigen/Dense>

#include "ai_server/util/math/geometry_traits.h"

namespace bg = boost::geometry;

using p_box_t = bg::model::box<bg::model::d2::point_xy<double>>;
using v_box_t = bg::model::box<Eigen::Vector2d>;

BOOST_AUTO_TEST_SUITE(math)

BOOST_AUTO_TEST_CASE(geometry_traits1, *boost::unit_test::tolerance(0.0000001)) {
  Eigen::Vector2d p{1.0, 3.0};

  const bg::traits::access<Eigen::Vector2d, 0> x;
  const bg::traits::access<Eigen::Vector2d, 1> y;

  // get()関数のテスト
  BOOST_TEST(x.get(p) == p.x());
  BOOST_TEST(y.get(p) == p.y());

  // set()関数のテスト
  x.set(p, -13.0);
  y.set(p, -17.0);
  BOOST_TEST(x.get(p) == -13.0);
  BOOST_TEST(y.get(p) == -17.0);
}

BOOST_AUTO_TEST_CASE(geometry_traits2, *boost::unit_test::tolerance(0.0000001)) {
  const p_box_t pbox{{1.0, 2.0}, {5.0, 11.0}};
  const v_box_t vbox{{1.0, 2.0}, {5.0, 11.0}};

  // get()関数のテスト
  BOOST_CHECK(bg::equals(pbox, vbox));

  // set()関数のテスト
  {
    p_box_t ret_pbox;
    v_box_t ret_vbox;

    // 座標変換する際にエラーがないことを確認する
    bg::strategy::transform::translate_transformer<double, 2, 2> translate(1.5, 1.5);
    BOOST_CHECK(bg::transform(pbox, ret_pbox, translate));
    BOOST_CHECK(bg::transform(vbox, ret_vbox, translate));

    // 値が正しいか
    BOOST_CHECK(bg::equals(ret_pbox, ret_vbox));
  }
}

BOOST_AUTO_TEST_SUITE_END()
