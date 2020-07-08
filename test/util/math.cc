#define BOOST_TEST_DYN_LINK

#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/ball.h"
#include "ai_server/model/robot.h"
#include "ai_server/util/math/affine.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

using namespace ai_server;

BOOST_AUTO_TEST_SUITE(math)

BOOST_AUTO_TEST_CASE(affine, *boost::unit_test::tolerance(0.0000001)) {
  // テスト用のオブジェクト
  model::ball b{100, 200, 0};
  model::robot r{300, 400, 0};

  auto t2d = std::make_tuple(100.0, 200.0);
  auto t3d = std::make_tuple(300.0, 400.0, 0.0);

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

    const auto t1 = util::math::transform(mat, t2d);
    BOOST_TEST(std::get<0>(t1) == std::get<0>(t2d));
    BOOST_TEST(std::get<1>(t1) == std::get<1>(t2d));

    const auto t2 = util::math::transform(mat, t3d);
    BOOST_TEST(std::get<0>(t2) == std::get<0>(t3d));
    BOOST_TEST(std::get<1>(t2) == std::get<1>(t3d));
    BOOST_TEST(std::get<2>(t2) == std::get<2>(t3d));
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

    const auto t1 = util::math::transform(mat, t2d);
    BOOST_TEST(std::get<0>(t1) == -std::get<1>(t2d) + 10);
    BOOST_TEST(std::get<1>(t1) == std::get<0>(t2d) + 20);

    const auto t2 = util::math::transform(mat, t3d);
    BOOST_TEST(std::get<0>(t2) == -std::get<1>(t3d) + 10);
    BOOST_TEST(std::get<1>(t2) == std::get<0>(t3d) + 20);
    BOOST_TEST(std::get<2>(t2) == -half_pi);
  }
}

BOOST_AUTO_TEST_CASE(to_vector) {
  model::robot r{};
  r.set_x(1);
  r.set_y(2);
  r.set_theta(3);
  r.set_vx(4);
  r.set_vy(5);
  r.set_omega(6);
  r.set_ax(7);
  r.set_ay(8);
  r.set_alpha(9);

  BOOST_TEST(util::math::position(r) == Eigen::Vector2d(1, 2));
  BOOST_TEST(util::math::position3d(r) == Eigen::Vector3d(1, 2, 3));

  BOOST_TEST(util::math::velocity(r) == Eigen::Vector2d(4, 5));
  BOOST_TEST(util::math::velocity3d(r) == Eigen::Vector3d(4, 5, 6));

  BOOST_TEST(util::math::acceleration(r) == Eigen::Vector2d(7, 8));
  BOOST_TEST(util::math::acceleration3d(r) == Eigen::Vector3d(7, 8, 9));
}

BOOST_AUTO_TEST_CASE(wrap_to_2pi, *boost::unit_test::tolerance(0.0000001)) {
  using namespace boost::math::double_constants;

  BOOST_TEST(util::math::wrap_to_2pi(0.0) == 0.0);
  BOOST_TEST(util::math::wrap_to_2pi(third_pi) == third_pi);
  BOOST_TEST(util::math::wrap_to_2pi(two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::math::wrap_to_2pi(pi) == pi);
  BOOST_TEST(util::math::wrap_to_2pi(pi + third_pi) == pi + third_pi);
  BOOST_TEST(util::math::wrap_to_2pi(pi + two_thirds_pi) == pi + two_thirds_pi);
  BOOST_TEST(util::math::wrap_to_2pi(two_pi) == 0.0);

  BOOST_TEST(util::math::wrap_to_2pi(-third_pi) == pi + two_thirds_pi);
  BOOST_TEST(util::math::wrap_to_2pi(-two_thirds_pi) == pi + third_pi);
  BOOST_TEST(util::math::wrap_to_2pi(-pi) == pi);
  BOOST_TEST(util::math::wrap_to_2pi(-pi - third_pi) == two_thirds_pi);
  BOOST_TEST(util::math::wrap_to_2pi(-pi - two_thirds_pi) == third_pi);

  BOOST_TEST(util::math::wrap_to_2pi(25 * two_pi + third_pi) == third_pi);
  BOOST_TEST(util::math::wrap_to_2pi(25 * two_pi + two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::math::wrap_to_2pi(25 * two_pi + pi) == pi);
  BOOST_TEST(util::math::wrap_to_2pi(-25 * two_pi + third_pi) == third_pi);
  BOOST_TEST(util::math::wrap_to_2pi(-25 * two_pi + two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::math::wrap_to_2pi(-25 * two_pi + pi) == pi);
}

BOOST_AUTO_TEST_CASE(wrap_to_pi, *boost::unit_test::tolerance(0.0000001)) {
  using namespace boost::math::double_constants;

  BOOST_TEST(util::math::wrap_to_pi(-two_thirds_pi) == -two_thirds_pi);
  BOOST_TEST(util::math::wrap_to_pi(-third_pi) == -third_pi);
  BOOST_TEST(util::math::wrap_to_pi(0.0) == 0.0);
  BOOST_TEST(util::math::wrap_to_pi(third_pi) == third_pi);
  BOOST_TEST(util::math::wrap_to_pi(two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::math::wrap_to_pi(pi) == pi);
  BOOST_TEST(util::math::wrap_to_pi(-pi) == pi);

  BOOST_TEST(util::math::wrap_to_pi(pi + third_pi) == -two_thirds_pi);
  BOOST_TEST(util::math::wrap_to_pi(pi + two_thirds_pi) == -third_pi);
  BOOST_TEST(util::math::wrap_to_pi(two_pi) == 0.0);

  BOOST_TEST(util::math::wrap_to_pi(-pi - third_pi) == two_thirds_pi);
  BOOST_TEST(util::math::wrap_to_pi(-pi - two_thirds_pi) == third_pi);

  BOOST_TEST(util::math::wrap_to_pi(25 * two_pi + third_pi) == third_pi);
  BOOST_TEST(util::math::wrap_to_pi(25 * two_pi + two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::math::wrap_to_pi(25 * two_pi + pi) == pi);
  BOOST_TEST(util::math::wrap_to_pi(-25 * two_pi + third_pi) == third_pi);
  BOOST_TEST(util::math::wrap_to_pi(-25 * two_pi + two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::math::wrap_to_pi(-25 * two_pi + pi) == pi);
}

BOOST_AUTO_TEST_CASE(delta_theta, *boost::unit_test::tolerance(0.0000001)) {
  using namespace boost::math::double_constants;

  BOOST_TEST(util::math::delta_theta(0.0, pi / 4) == pi / 4);
  BOOST_TEST(util::math::delta_theta(pi / 8, -pi / 8) == -pi / 4);
  BOOST_TEST(util::math::delta_theta(pi / 4, -pi / 4) == -half_pi);
  BOOST_TEST(util::math::delta_theta(pi / 3, -pi / 3) == -two_thirds_pi);
  BOOST_TEST(util::math::delta_theta(-half_pi, half_pi) == pi);
  BOOST_TEST(util::math::delta_theta(half_pi, -half_pi) == -pi);
  BOOST_TEST(util::math::delta_theta(two_pi, 0.0) == 0.0);
  BOOST_TEST(util::math::delta_theta(two_thirds_pi, 4.0 * third_pi) == third_pi * 2.0);
  BOOST_TEST(util::math::delta_theta(two_pi * 10 + two_thirds_pi, 4.0 * third_pi) ==
             third_pi * 2.0);
}

BOOST_AUTO_TEST_CASE(theta_ave, *boost::unit_test::tolerance(0.0000001)) {
  using namespace boost::math::double_constants;
  using vec_t = std::vector<double>;

  // 空
  vec_t v0{};
  BOOST_TEST(util::math::theta_average(v0.begin(), v0.end()) == 0.0);

  vec_t v1{1.0};
  BOOST_TEST(util::math::theta_average(v1.begin(), v1.end()) == 1.0);

  vec_t v2{1.0, -1.0};
  BOOST_TEST(util::math::theta_average(v2.begin(), v2.end()) == 0.0);

  vec_t v3{pi / 4.0, 7.0 * pi / 4.0};
  BOOST_TEST(util::math::theta_average(v3.begin(), v3.end()) == 0.0);

  vec_t v4{0.0, 2.0 * pi, 4.0 * pi, 6.0 * pi};
  BOOST_TEST(util::math::theta_average(v4.begin(), v4.end()) == 0.0);

  vec_t v5{-pi, pi, 3.0 * pi};
  BOOST_TEST(util::math::theta_average(v5.begin(), v5.end()) == pi);
}

BOOST_AUTO_TEST_SUITE_END()
