#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE command_test
#include <boost/test/unit_test.hpp>
#include "ai_server/model/command.h"
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::command::kick_type_t)

BOOST_AUTO_TEST_SUITE(command)

// command getter and initialization check
BOOST_AUTO_TEST_CASE(test01) {
  ai_server::model::command command{0};

  BOOST_TEST(command.id() == 0);
  BOOST_TEST(command.dribble() == 0);
  BOOST_TEST(std::get<0>(command.kick_flag()) == ai_server::model::command::kick_type_t::none);
  BOOST_TEST(std::get<1>(command.kick_flag()) == 0.0);
  const ai_server::model::command::position_t& test_position =
      boost::get<ai_server::model::command::position_t>(command.setpoint());
  BOOST_TEST(test_position.x == 0.0);
  BOOST_TEST(test_position.y == 0.0);
  BOOST_TEST(test_position.theta == 0.0);
}

// command setter check
BOOST_AUTO_TEST_CASE(test02) {
  ai_server::model::command command{0};

  command.set_dribble(1);
  BOOST_TEST(command.dribble() == 1);
  ai_server::model::command::kick_flag_t kick_flag_test(
      ai_server::model::command::kick_type_t::line, 2.0);
  command.set_kick_flag(kick_flag_test);
  BOOST_TEST(std::get<0>(command.kick_flag()) == ai_server::model::command::kick_type_t::line);
  BOOST_TEST(std::get<1>(command.kick_flag()) == 2.0);
  ai_server::model::command::position_t position{3.0, 4.0, 5.0};
  command.set_position(position);
  const ai_server::model::command::position_t& test_position =
      boost::get<ai_server::model::command::position_t>(command.setpoint());
  BOOST_TEST(test_position.x == 3.0);
  BOOST_TEST(test_position.y == 4.0);
  BOOST_TEST(test_position.theta == 5.0);
  ai_server::model::command::velocity_t velocity{6.0, 7.0, 8.0};
  command.set_velocity(velocity);
  const ai_server::model::command::velocity_t& test_velocity =
      boost::get<ai_server::model::command::velocity_t>(command.setpoint());
  BOOST_TEST(test_velocity.vx == 6.0);
  BOOST_TEST(test_velocity.vy == 7.0);
  BOOST_TEST(test_velocity.omega == 8.0);
}

BOOST_AUTO_TEST_SUITE_END()
