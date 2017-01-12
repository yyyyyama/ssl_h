#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE command_test
#include <boost/test/unit_test.hpp>
#include "ai_server/model/command.h"

BOOST_AUTO_TEST_SUITE(command)

// command getter and initialization check
BOOST_AUTO_TEST_CASE(test01) {
  ai_server::model::command command{0};

  BOOST_TEST(command.id() == 0);
  BOOST_TEST(command.kick() == 0);
  BOOST_TEST(command.chip() == 0);
  BOOST_TEST(command.dribble() == 0);
  const ai_server::model::position_t& test_position =
      boost::get<ai_server::model::position_t>(command.setpoint());
  BOOST_TEST(test_position.x == 0.0);
  BOOST_TEST(test_position.y == 0.0);
  BOOST_TEST(test_position.theta == 0.0);
}

// command setter check
BOOST_AUTO_TEST_CASE(test02) {
  ai_server::model::command command{0};

  command.set_kick(1);
  BOOST_TEST(command.kick() == 1);
  command.set_chip(2);
  BOOST_TEST(command.chip() == 2);
  command.set_dribble(3);
  BOOST_TEST(command.dribble() == 3);
  ai_server::model::position_t position{4.0, 5.0, 6.0};
  command.set_position(position);
  const ai_server::model::position_t& test_position =
      boost::get<ai_server::model::position_t>(command.setpoint());
  BOOST_TEST(test_position.x == 4.0);
  BOOST_TEST(test_position.y == 5.0);
  BOOST_TEST(test_position.theta == 6.0);
  ai_server::model::velocity_t velocity{7.0, 8.0, 9.0};
  command.set_velocity(velocity);
  const ai_server::model::velocity_t& test_velocity =
      boost::get<ai_server::model::velocity_t>(command.setpoint());
  BOOST_TEST(test_velocity.vx == 7.0);
  BOOST_TEST(test_velocity.vy == 8.0);
  BOOST_TEST(test_velocity.omega == 9.0);
}

BOOST_AUTO_TEST_SUITE_END()
