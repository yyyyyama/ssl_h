#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai_server/util/math/geometry.h"

using namespace ai_server;

BOOST_AUTO_TEST_SUITE(geometry)

BOOST_AUTO_TEST_CASE(test001, *boost::unit_test::tolerance(0.0000001)) {
  const auto tmp1 = util::math::calc_isosceles_vertexes(Eigen::Vector2d{0.0, 0.0},
                                                        Eigen::Vector2d{0.0, 1.0}, 1.0);
  BOOST_TEST(std::get<0>(tmp1).x() == -1.0);
  BOOST_TEST(std::get<0>(tmp1).y() == 1.0);
  BOOST_TEST(std::get<1>(tmp1).x() == 1.0);
  BOOST_TEST(std::get<1>(tmp1).y() == 1.0);

  const auto tmp2 = util::math::calc_isosceles_vertexes(Eigen::Vector2d{0.0, 1.0},
                                                        Eigen::Vector2d{0.0, 0.0}, 1.0);
  BOOST_TEST(std::get<0>(tmp2).x() == 1.0);
  BOOST_TEST(std::get<0>(tmp2).y() == 0.0);
  BOOST_TEST(std::get<1>(tmp2).x() == -1.0);
  BOOST_TEST(std::get<1>(tmp2).y() == 0.0);
}

BOOST_AUTO_TEST_SUITE_END()