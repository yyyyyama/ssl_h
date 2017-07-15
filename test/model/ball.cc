#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai_server/model/ball.h"

BOOST_AUTO_TEST_SUITE(ball_datatype)

BOOST_AUTO_TEST_CASE(test01) {
  // constructor
  ai_server::model::ball b{1.0, 2.0, 3.0};

  // get value
  BOOST_TEST(b.x() == 1.0);
  BOOST_TEST(b.y() == 2.0);
  BOOST_TEST(b.z() == 3.0);
}

BOOST_AUTO_TEST_CASE(test02) {
  ai_server::model::ball b{};

  // init
  BOOST_TEST(b.x() == 0.0);
  BOOST_TEST(b.y() == 0.0);
  BOOST_TEST(b.z() == 0.0);

  // set x
  b.set_x(4.0);
  BOOST_TEST(b.x() == 4.0);
  // set y
  b.set_y(5.0);
  BOOST_TEST(b.y() == 5.0);
  // set z
  b.set_z(6.0);
  BOOST_TEST(b.z() == 6.0);
  // set vx
  b.set_vx(7.0);
  BOOST_TEST(b.vx() == 7.0);
  // set vy
  b.set_vy(8.0);
  BOOST_TEST(b.vy() == 8.0);
  // set ax
  b.set_ax(9.0);
  BOOST_TEST(b.ax() == 9.0);
  // set ay
  b.set_ay(10.0);
  BOOST_TEST(b.ay() == 10.0);
}

BOOST_AUTO_TEST_SUITE_END()
