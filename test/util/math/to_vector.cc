#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai_server/model/ball.h"
#include "ai_server/model/robot.h"
#include "ai_server/util/math/to_vector.h"

using namespace ai_server;

BOOST_AUTO_TEST_SUITE(to_vector)

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

BOOST_AUTO_TEST_SUITE_END()
