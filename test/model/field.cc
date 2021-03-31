#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai_server/model/field.h"

BOOST_AUTO_TEST_SUITE(field_data)

BOOST_AUTO_TEST_CASE(test001) {
  ai_server::model::field f{};

  BOOST_TEST(f.length() == 4050);
  BOOST_TEST(f.width() == 3025);
  BOOST_TEST(f.center_radius() == 500);
  BOOST_TEST(f.goal_width() == 1000);
  BOOST_TEST(f.penalty_length() == 600);
  BOOST_TEST(f.penalty_width() == 1600);
}
BOOST_AUTO_TEST_CASE(test002) {
  ai_server::model::field f{};

  f.set_length(1);
  f.set_width(2);
  f.set_center_radius(3);
  f.set_goal_width(4);
  f.set_penalty_length(5);
  f.set_penalty_width(6);

  BOOST_TEST(f.length() == 1);
  BOOST_TEST(f.width() == 2);
  BOOST_TEST(f.center_radius() == 3);
  BOOST_TEST(f.goal_width() == 4);
  BOOST_TEST(f.penalty_length() == 5);
  BOOST_TEST(f.penalty_width() == 6);
}

BOOST_AUTO_TEST_CASE(test003, *boost::unit_test::tolerance(0.0000001)) {
  ai_server::model::field f{};

  f.set_length(100);
  f.set_width(90);
  f.set_goal_width(40);
  f.set_penalty_length(30);
  f.set_penalty_width(20);

  BOOST_TEST(f.x_max() == 50);
  BOOST_TEST(f.x_min() == -50);
  BOOST_TEST(f.y_max() == 45);
  BOOST_TEST(f.y_min() == -45);

  /////////////////
  BOOST_TEST(f.back_penalty_x() == -50.0 + 30.0);
  BOOST_TEST(f.front_penalty_x() == 50.0 - 30.0);
  BOOST_TEST(f.penalty_y_max() == 10.0);
  BOOST_TEST(f.penalty_y_min() == -10.0);
  BOOST_TEST(f.goal_y_max() == 20.0);
  BOOST_TEST(f.goal_y_min() == -20.0);

  /////////////////
  BOOST_TEST(f.back_goal_center().x == -50.0);
  BOOST_TEST(f.back_goal_center().y == 0.0);
  BOOST_TEST(f.back_goal_left().x == -50.0);
  BOOST_TEST(f.back_goal_left().y == 20.0);
  BOOST_TEST(f.back_goal_right().x == -50.0);
  BOOST_TEST(f.back_goal_right().y == -20.0);
  // ---
  BOOST_TEST(f.front_goal_center().x == 50.0);
  BOOST_TEST(f.front_goal_center().y == 0.0);
  BOOST_TEST(f.front_goal_left().x == 50.0);
  BOOST_TEST(f.front_goal_left().y == 20.0);
  BOOST_TEST(f.front_goal_right().x == 50.0);
  BOOST_TEST(f.front_goal_right().y == -20.0);

  /////////////////
  BOOST_TEST(f.back_left_corner().x == -50.0);
  BOOST_TEST(f.back_left_corner().y == 45.0);
  BOOST_TEST(f.back_right_corner().x == -50.0);
  BOOST_TEST(f.back_right_corner().y == -45.0);
  // ---
  BOOST_TEST(f.front_left_corner().x == 50.0);
  BOOST_TEST(f.front_left_corner().y == 45.0);
  BOOST_TEST(f.front_right_corner().x == 50.0);
  BOOST_TEST(f.front_right_corner().y == -45.0);

  /////////////////
  BOOST_TEST(f.back_penalty_mark().x == 50.0 - 700.0);
  BOOST_TEST(f.back_penalty_mark().y == 0.0);
  BOOST_TEST(f.front_penalty_mark().x == -50.0 + 700.0);
  BOOST_TEST(f.front_penalty_mark().y == 0.0);

  /////////////////
  BOOST_TEST(f.back_half_area().max.x == 0.0);
  BOOST_TEST(f.back_half_area().max.y == 45.0);
  BOOST_TEST(f.back_half_area().min.x == -50.0);
  BOOST_TEST(f.back_half_area().min.y == -45.0);
  // ---
  BOOST_TEST(f.front_half_area().max.x == 50.0);
  BOOST_TEST(f.front_half_area().max.y == 45.0);
  BOOST_TEST(f.front_half_area().min.x == 0.0);
  BOOST_TEST(f.front_half_area().min.y == -45.0);

  /////////////////
  BOOST_TEST(f.back_penalty_area().max.x == -50.0 + 30.0);
  BOOST_TEST(f.back_penalty_area().max.y == 10.0);
  BOOST_TEST(f.back_penalty_area().min.x == -50.0);
  BOOST_TEST(f.back_penalty_area().min.y == -10.0);
  // ---
  BOOST_TEST(f.front_penalty_area().max.x == 50.0);
  BOOST_TEST(f.front_penalty_area().max.y == 10.0);
  BOOST_TEST(f.front_penalty_area().min.x == 50.0 - 30.0);
  BOOST_TEST(f.front_penalty_area().min.y == -10.0);

  /////////////////
  BOOST_TEST(f.game_area().max.x == 50.0);
  BOOST_TEST(f.game_area().max.y == 45.0);
  BOOST_TEST(f.game_area().min.x == -50.0);
  BOOST_TEST(f.game_area().min.y == -45.0);
}

BOOST_AUTO_TEST_CASE(test004) {
  ai_server::model::field f{};

  f.set_length(1);
  f.set_width(2);
  f.set_center_radius(3);
  f.set_goal_width(4);
  f.set_penalty_length(5);
  f.set_penalty_width(6);

  const ai_server::model::field& fc = f;
  BOOST_TEST(fc.length() == 1);
  BOOST_TEST(fc.width() == 2);
  BOOST_TEST(fc.center_radius() == 3);
  BOOST_TEST(fc.goal_width() == 4);
  BOOST_TEST(fc.penalty_length() == 5);
  BOOST_TEST(fc.penalty_width() == 6);
}

BOOST_AUTO_TEST_SUITE_END()
