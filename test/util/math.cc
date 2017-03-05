#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE util_math_test

#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/util/math.h"

using namespace ai_server;

BOOST_AUTO_TEST_SUITE(math)

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
  BOOST_TEST(util::wrap_to_2pi(25 * two_pi +  pi) == pi);
  BOOST_TEST(util::wrap_to_2pi(-25 * two_pi + third_pi) == third_pi);
  BOOST_TEST(util::wrap_to_2pi(-25 * two_pi + two_thirds_pi) == two_thirds_pi);
  BOOST_TEST(util::wrap_to_2pi(-25 * two_pi +  pi) == pi);
}

BOOST_AUTO_TEST_SUITE_END()
