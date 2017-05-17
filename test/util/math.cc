#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE util_math_test

#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/command.h"
#include "ai_server/model/robot.h"
#include "ai_server/util/math.h"
#include "ai_server/util/math/to_vector.h"

using namespace ai_server;

BOOST_AUTO_TEST_SUITE(math)

BOOST_AUTO_TEST_CASE(to_vector) {
  // 各メタ関数のテスト
  // model::robotはx(), y(), vx(), vy(), ax(), ay()を持っているのでtrue
  static_assert(util::math::detail::has_x_y_v<model::robot>, "");
  static_assert(util::math::detail::has_vx_vy_v<model::robot>, "");
  static_assert(util::math::detail::has_ax_ay_v<model::robot>, "");
  // model::commandはx(), y(), vx(), vy(), ax(), ay()を持っていないのでfalse
  static_assert(!util::math::detail::has_x_y_v<model::command>, "");
  static_assert(!util::math::detail::has_vx_vy_v<model::command>, "");
  static_assert(!util::math::detail::has_ax_ay_v<model::command>, "");

  model::robot r{};
  r.set_x(1);
  r.set_y(2);
  r.set_vx(3);
  r.set_vy(4);
  r.set_ax(5);
  r.set_ay(6);

  BOOST_TEST(util::math::position(r) == Eigen::Vector2d(1, 2));
  BOOST_TEST(util::math::velocity(r) == Eigen::Vector2d(3, 4));
  BOOST_TEST(util::math::acceleration(r) == Eigen::Vector2d(5, 6));
}

BOOST_AUTO_TEST_CASE(wrap_to_2pi, *boost::unit_test::tolerance(0.0000001)) {
  using namespace boost::math::double_constants;

  BOOST_TEST(util::wrap_to_2pi(0.0) == 0.0);
  BOOST_TEST(util::wrap_to_2pi(third_pi) == third_pi);
  BOOST_TEST(util::wrap_to_2pi(two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::wrap_to_2pi(pi) == pi);
  BOOST_TEST(util::wrap_to_2pi(pi + third_pi) == pi + third_pi);
  BOOST_TEST(util::wrap_to_2pi(pi + two_thirds_pi) == pi + two_thirds_pi);
  BOOST_TEST(util::wrap_to_2pi(two_pi) == 0.0);

  BOOST_TEST(util::wrap_to_2pi(-third_pi) == pi + two_thirds_pi);
  BOOST_TEST(util::wrap_to_2pi(-two_thirds_pi) == pi + third_pi);
  BOOST_TEST(util::wrap_to_2pi(-pi) == pi);
  BOOST_TEST(util::wrap_to_2pi(-pi - third_pi) == two_thirds_pi);
  BOOST_TEST(util::wrap_to_2pi(-pi - two_thirds_pi) == third_pi);

  BOOST_TEST(util::wrap_to_2pi(25 * two_pi + third_pi) == third_pi);
  BOOST_TEST(util::wrap_to_2pi(25 * two_pi + two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::wrap_to_2pi(25 * two_pi + pi) == pi);
  BOOST_TEST(util::wrap_to_2pi(-25 * two_pi + third_pi) == third_pi);
  BOOST_TEST(util::wrap_to_2pi(-25 * two_pi + two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::wrap_to_2pi(-25 * two_pi + pi) == pi);
}

BOOST_AUTO_TEST_CASE(wrap_to_pi, *boost::unit_test::tolerance(0.0000001)) {
  using namespace boost::math::double_constants;

  BOOST_TEST(util::wrap_to_pi(-two_thirds_pi) == -two_thirds_pi);
  BOOST_TEST(util::wrap_to_pi(-third_pi) == -third_pi);
  BOOST_TEST(util::wrap_to_pi(0.0) == 0.0);
  BOOST_TEST(util::wrap_to_pi(third_pi) == third_pi);
  BOOST_TEST(util::wrap_to_pi(two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::wrap_to_pi(pi) == pi);
  BOOST_TEST(util::wrap_to_pi(-pi) == pi);

  BOOST_TEST(util::wrap_to_pi(pi + third_pi) == -two_thirds_pi);
  BOOST_TEST(util::wrap_to_pi(pi + two_thirds_pi) == -third_pi);
  BOOST_TEST(util::wrap_to_pi(two_pi) == 0.0);

  BOOST_TEST(util::wrap_to_pi(-pi - third_pi) == two_thirds_pi);
  BOOST_TEST(util::wrap_to_pi(-pi - two_thirds_pi) == third_pi);

  BOOST_TEST(util::wrap_to_pi(25 * two_pi + third_pi) == third_pi);
  BOOST_TEST(util::wrap_to_pi(25 * two_pi + two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::wrap_to_pi(25 * two_pi + pi) == pi);
  BOOST_TEST(util::wrap_to_pi(-25 * two_pi + third_pi) == third_pi);
  BOOST_TEST(util::wrap_to_pi(-25 * two_pi + two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::wrap_to_pi(-25 * two_pi + pi) == pi);
}

BOOST_AUTO_TEST_SUITE_END()
