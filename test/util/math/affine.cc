#define BOOST_TEST_DYN_LINK

#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/ball.h"
#include "ai_server/model/robot.h"
#include "ai_server/util/math/affine.h"

using namespace ai_server;

BOOST_AUTO_TEST_SUITE(affine)

BOOST_AUTO_TEST_CASE(affine, *boost::unit_test::tolerance(0.0000001)) {
  // テスト用のオブジェクト
  model::ball b{100, 200, 0};
  model::robot r{300, 400, 0};
  b.set_vx(100.0);
  b.set_vy(200.0);
  b.set_ax(300.0);
  b.set_ay(400.0);
  r.set_vx(500.0);
  r.set_vy(600.0);
  r.set_omega(700.0);
  r.set_ax(800.0);
  r.set_ay(900.0);
  r.set_alpha(1000.0);

  Eigen::Vector2d v2d(100.0, 200.0);

  {
    // 何もしない変換
    const auto mat = util::math::make_transformation_matrix(.0, .0, .0);

    const auto b2 = util::math::transform(mat, b);
    BOOST_TEST(b2.x() == b.x());
    BOOST_TEST(b2.y() == b.y());

    const auto r2 = util::math::transform(mat, r);
    BOOST_TEST(r2.x() == r.x());
    BOOST_TEST(r2.y() == r.y());
    BOOST_TEST(r2.theta() == r.theta());

    const auto b3 = util::math::transform_all(mat, b);
    BOOST_TEST(b3.x() == b.x());
    BOOST_TEST(b3.y() == b.y());
    BOOST_TEST(b3.vx() == b.vx());
    BOOST_TEST(b3.vy() == b.vy());
    BOOST_TEST(b3.ax() == b.ax());
    BOOST_TEST(b3.ay() == b.ay());

    const auto r3 = util::math::transform_all(mat, r);
    BOOST_TEST(r3.x() == r.x());
    BOOST_TEST(r3.y() == r.y());
    BOOST_TEST(r3.theta() == r.theta());
    BOOST_TEST(r3.vx() == r.vx());
    BOOST_TEST(r3.vy() == r.vy());
    BOOST_TEST(r3.omega() == r.omega());
    BOOST_TEST(r3.ax() == r.ax());
    BOOST_TEST(r3.ay() == r.ay());
    BOOST_TEST(r3.alpha() == r.alpha());

    const Eigen::Vector2d v1 = util::math::transform(mat, v2d);
    BOOST_TEST(v1.x() == v2d.x());
    BOOST_TEST(v1.y() == v2d.y());
  }

  {
    // 90度回転, x軸方向に10, y軸方向に20平行移動
    using namespace boost::math::double_constants;
    const auto mat = util::math::make_transformation_matrix(10.0, 20.0, half_pi);

    const auto b2 = util::math::transform(mat, b);
    BOOST_TEST(b2.x() == -b.y() + 10);
    BOOST_TEST(b2.y() == b.x() + 20);

    const auto r2 = util::math::transform(mat, r);
    BOOST_TEST(r2.x() == -r.y() + 10);
    BOOST_TEST(r2.y() == r.x() + 20);
    BOOST_TEST(r2.theta() == 3 * half_pi);

    const auto b3 = util::math::transform_all(mat, b);
    BOOST_TEST(b3.x() == -b.y() + 10);
    BOOST_TEST(b3.y() == b.x() + 20);
    BOOST_TEST(b3.vx() == -b.vy());
    BOOST_TEST(b3.vy() == b.vx());
    BOOST_TEST(b3.ax() == -b.ay());
    BOOST_TEST(b3.ay() == b.ax());

    const auto r3 = util::math::transform_all(mat, r);
    BOOST_TEST(r3.x() == -r.y() + 10);
    BOOST_TEST(r3.y() == r.x() + 20);
    BOOST_TEST(r3.theta() == 3 * half_pi);
    BOOST_TEST(r3.vx() == -r.vy());
    BOOST_TEST(r3.vy() == r.vx());
    BOOST_TEST(r3.omega() == r.omega());
    BOOST_TEST(r3.ax() == -r.ay());
    BOOST_TEST(r3.ay() == r.ax());
    BOOST_TEST(r3.alpha() == r.alpha());

    const Eigen::Vector2d v1 = util::math::transform(mat, v2d);
    BOOST_TEST(v1.x() == -v2d.y() + 10);
    BOOST_TEST(v1.y() == v2d.x() + 20);
  }
}

BOOST_AUTO_TEST_SUITE_END()
