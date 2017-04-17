#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE coordinate_transformation_test

#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>
#include <chrono>
#include <cmath>

#include "ai_server/filter/coordinate_transformation.h"
#include "ai_server/model/ball.h"
#include "ai_server/model/robot.h"

BOOST_AUTO_TEST_SUITE(coordinate_transformation)

BOOST_AUTO_TEST_CASE(test01, *boost::unit_test::tolerance(0.0005)) {
  using boost::math::constants::pi;
  using namespace std::chrono_literals;

  ai_server::filter::coordinate_transformation<ai_server::model::ball> test_transformation(
      100, 100, pi<double>());
  ai_server::model::ball test_ball{100, 100, 0.0};
  const auto t1 = std::chrono::high_resolution_clock::time_point{};

  test_transformation.apply(test_ball, t1);

  BOOST_TEST(test_ball.x() == 0.0);
  BOOST_TEST(test_ball.y() == 0.0);
}

BOOST_AUTO_TEST_CASE(test02, *boost::unit_test::tolerance(0.0005)) {
  using boost::math::constants::pi;
  using namespace std::chrono_literals;

  ai_server::filter::coordinate_transformation<ai_server::model::robot> test_transformation(
      100, 100, pi<double>());
  ai_server::model::robot test_robot{100, 100, 0.0};
  const auto t1 = std::chrono::high_resolution_clock::time_point{};

  test_transformation.apply(test_robot, t1);

  BOOST_TEST(test_robot.x() == 0.0);
  BOOST_TEST(test_robot.y() == 0.0);
  BOOST_TEST(test_robot.theta == pi<double>());
}

BOOST_AUTO_TEST_SUITE_END()
