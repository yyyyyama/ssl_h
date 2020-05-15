#define BOOST_TEST_DYN_LINK

#include <variant>
#include <boost/test/unit_test.hpp>
#include "ai_server/model/command.h"
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::command::kick_type_t)

BOOST_AUTO_TEST_SUITE(command)

// command getter and initialization check
BOOST_AUTO_TEST_CASE(test01) {
  ai_server::model::command command{0};

  BOOST_TEST(command.id() == 0);
  BOOST_TEST(command.dribble() == 0);
  auto kick_flag_test = command.kick_flag();
  BOOST_TEST(std::get<0>(kick_flag_test) == ai_server::model::command::kick_type_t::none);
  BOOST_TEST(std::get<1>(kick_flag_test) == 0.0);
  const auto test_velocity =
      std::get_if<ai_server::model::command::velocity_t>(&command.setpoint());
  BOOST_TEST(test_velocity != nullptr);
  BOOST_TEST(test_velocity->vx == 0.0);
  BOOST_TEST(test_velocity->vy == 0.0);
  BOOST_TEST(test_velocity->omega == 0.0);
}

// command setter check
BOOST_AUTO_TEST_CASE(test02) {
  ai_server::model::command command{0};

  command.set_dribble(1);
  BOOST_TEST(command.dribble() == 1);
  ai_server::model::command::kick_flag_t kick_flag_test(
      ai_server::model::command::kick_type_t::line, 2.0);
  command.set_kick_flag(kick_flag_test);
  auto kick_flag_test2 = command.kick_flag();
  BOOST_TEST(std::get<0>(kick_flag_test2) == ai_server::model::command::kick_type_t::line);
  BOOST_TEST(std::get<1>(kick_flag_test2) == 2.0);
  ai_server::model::command::position_t position{3.0, 4.0, 5.0};
  command.set_position(position);
  const auto test_position =
      std::get_if<ai_server::model::command::position_t>(&command.setpoint());
  BOOST_TEST(test_position != nullptr);
  BOOST_TEST(test_position->x == 3.0);
  BOOST_TEST(test_position->y == 4.0);
  BOOST_TEST(test_position->theta == 5.0);
  ai_server::model::command::velocity_t velocity{6.0, 7.0, 8.0};
  command.set_velocity(velocity);
  const auto test_velocity =
      std::get_if<ai_server::model::command::velocity_t>(&command.setpoint());
  BOOST_TEST(test_velocity != nullptr);
  BOOST_TEST(test_velocity->vx == 6.0);
  BOOST_TEST(test_velocity->vy == 7.0);
  BOOST_TEST(test_velocity->omega == 8.0);
}

BOOST_AUTO_TEST_CASE(temp_if) {
  ai_server::model::command cmd{0};

  // 座標と角速度が設定された状態で setpoint() を呼んだら例外
  cmd.set_position(123, 456);
  cmd.set_velanglar(789);
  BOOST_CHECK_THROW(cmd.setpoint(), std::runtime_error);

  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::position>(sp);
    BOOST_TEST(std::get<0>(r1) == 123);
    BOOST_TEST(std::get<1>(r1) == 456);

    const auto r2 = std::get<ai_server::model::setpoint::velangular>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 789);
  }

  // 速度と角度が設定された状態で setpoint() を呼んだら例外
  cmd.set_velocity(321, 654);
  cmd.set_angle(987);
  BOOST_CHECK_THROW(cmd.setpoint(), std::runtime_error);

  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::velocity>(sp);
    BOOST_TEST(std::get<0>(r1) == 321);
    BOOST_TEST(std::get<1>(r1) == 654);

    const auto r2 = std::get<ai_server::model::setpoint::angle>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 987);
  }
}

BOOST_AUTO_TEST_SUITE_END()
