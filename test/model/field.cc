#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE field_data_test

#include <boost/test/unit_test.hpp>

#include "ai_server/model/field.h"

BOOST_AUTO_TEST_SUITE(field_data)

BOOST_AUTO_TEST_CASE(test001) {
  ai_server::model::field f{};

  BOOST_TEST(f.length() == 9000);
  BOOST_TEST(f.width() == 6000);
  BOOST_TEST(f.center_radius() == 500);
  BOOST_TEST(f.goal_width() == 1000);
  BOOST_TEST(f.penalty_radius() == 1000);
  BOOST_TEST(f.penalty_line_length() == 500);
}
BOOST_AUTO_TEST_CASE(test002) {
  ai_server::model::field f{};

  f.set_length(1);
  f.set_width(2);
  f.set_center_radius(3);
  f.set_goal_width(4);
  f.set_penalty_radius(5);
  f.set_penalty_line_length(6);

  BOOST_TEST(f.length() == 1);
  BOOST_TEST(f.width() == 2);
  BOOST_TEST(f.center_radius() == 3);
  BOOST_TEST(f.goal_width() == 4);
  BOOST_TEST(f.penalty_radius() == 5);
  BOOST_TEST(f.penalty_line_length() == 6);
}

BOOST_AUTO_TEST_CASE(test003) {
  ai_server::model::field f{};

  f.set_length(100);
  f.set_width(90);

  BOOST_TEST(f.x_max() == 50);
  BOOST_TEST(f.x_min() == -50);
  BOOST_TEST(f.y_max() == 45);
  BOOST_TEST(f.y_min() == -45);
}

BOOST_AUTO_TEST_CASE(test004) {
  ai_server::model::field f{};

  f.set_length(1);
  f.set_width(2);
  f.set_center_radius(3);
  f.set_goal_width(4);
  f.set_penalty_radius(5);
  f.set_penalty_line_length(6);

  const ai_server::model::field& fc = f;
  BOOST_TEST(fc.length() == 1);
  BOOST_TEST(fc.width() == 2);
  BOOST_TEST(fc.center_radius() == 3);
  BOOST_TEST(fc.goal_width() == 4);
  BOOST_TEST(fc.penalty_radius() == 5);
  BOOST_TEST(fc.penalty_line_length() == 6);
}

BOOST_AUTO_TEST_SUITE_END()
