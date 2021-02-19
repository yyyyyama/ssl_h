#define BOOST_TEST_DYN_LINK

#include <vector>
#include <boost/math/constants/constants.hpp>
#include <boost/test/data/test_case.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/util/math/angle.h"

using namespace ai_server;

BOOST_AUTO_TEST_SUITE(angle)

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

BOOST_TEST_DECORATOR(*boost::unit_test::tolerance(0.0000001))
BOOST_DATA_TEST_CASE(inverse_angle_arg, boost::unit_test::data::make({-5, -1, 0, 5}), cycle) {
  using namespace boost::math::double_constants;

  const double offset = cycle * two_pi;

  BOOST_TEST(util::math::inverse(offset + sixth_pi) == -pi + sixth_pi);
  BOOST_TEST(util::math::inverse(offset + half_pi + sixth_pi) == -half_pi + sixth_pi);
  BOOST_TEST(util::math::inverse(offset + pi + sixth_pi) == sixth_pi);
  BOOST_TEST(util::math::inverse(offset + pi + half_pi + sixth_pi) == half_pi + sixth_pi);
}

BOOST_DATA_TEST_CASE(complare_arg,
                     boost::unit_test::data::make({-5, 0, 5}) *
                         boost::unit_test::data::make({-2, -1, 0, 2}),
                     common_cycle, cycle) {
  using namespace boost::math::double_constants;

  const double from   = (common_cycle + cycle) * two_pi + sixth_pi;
  const double offset = common_cycle * two_pi + third_pi;

  BOOST_CHECK(util::math::left_of(offset, from));
  BOOST_CHECK(!util::math::right_of(offset, from));

  BOOST_CHECK(util::math::left_of(offset + half_pi, from));
  BOOST_CHECK(!util::math::right_of(offset + half_pi, from));

  BOOST_CHECK(!util::math::left_of(offset + pi, from));
  BOOST_CHECK(util::math::right_of(offset + pi, from));

  BOOST_CHECK(!util::math::left_of(offset + pi + half_pi, from));
  BOOST_CHECK(util::math::right_of(offset + pi + half_pi, from));
}

BOOST_AUTO_TEST_SUITE_END()
