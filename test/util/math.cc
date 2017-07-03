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

  const auto p = model::command::position_t{12, 34, 56};
  BOOST_TEST(util::math::position(p) == Eigen::Vector2d(12, 34));
  BOOST_TEST(util::math::position3d(p) == Eigen::Vector3d(12, 34, 56));

  const auto v = model::command::velocity_t{56, 78, 90};
  BOOST_TEST(util::math::velocity(v) == Eigen::Vector2d(56, 78));
  BOOST_TEST(util::math::velocity3d(v) == Eigen::Vector3d(56, 78, 90));

  const auto a = model::command::acceleration_t{90, 12, 34};
  BOOST_TEST(util::math::acceleration(a) == Eigen::Vector2d(90, 12));
  BOOST_TEST(util::math::acceleration3d(a) == Eigen::Vector3d(90, 12, 34));
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
