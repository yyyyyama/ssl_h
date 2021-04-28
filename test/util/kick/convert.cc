#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai_server/util/kick/convert.h"

using namespace ai_server;

BOOST_AUTO_TEST_SUITE(kick_utils)

BOOST_AUTO_TEST_CASE(convert, *boost::unit_test::tolerance(0.0000001)) {
  BOOST_TEST(util::kick::power_to_speed(60) == 6000.0);
  BOOST_TEST(util::kick::power_to_speed(100) == 8000.0);
  BOOST_TEST(util::kick::power_to_speed(-10) == 0.0);
  BOOST_TEST(util::kick::speed_to_power(6000.0) == 60);
  BOOST_TEST(util::kick::speed_to_power(100000.0) == 255);
  BOOST_TEST(util::kick::speed_to_power(-100.0) == 0);
}

BOOST_AUTO_TEST_SUITE_END()