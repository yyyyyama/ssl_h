#define BOOST_TEST_DYN_LINK

#include <variant>
#include <boost/test/unit_test.hpp>
#include "ai_server/model/command.h"
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::command::kick_type_t)

BOOST_AUTO_TEST_SUITE(command)

// command getter and initialization check
BOOST_AUTO_TEST_CASE(test01) {
  ai_server::model::command command{};

  BOOST_TEST(command.dribble() == 0);
  auto kick_flag_test = command.kick_flag();
  BOOST_TEST(std::get<0>(kick_flag_test) == ai_server::model::command::kick_type_t::none);
  BOOST_TEST(std::get<1>(kick_flag_test) == 0.0);
}

// command setter check
BOOST_AUTO_TEST_CASE(test02) {
  ai_server::model::command command{};

  command.set_dribble(1);
  BOOST_TEST(command.dribble() == 1);
  ai_server::model::command::kick_flag_t kick_flag_test(
      ai_server::model::command::kick_type_t::line, 2.0);
  command.set_kick_flag(kick_flag_test);
  auto kick_flag_test2 = command.kick_flag();
  BOOST_TEST(std::get<0>(kick_flag_test2) == ai_server::model::command::kick_type_t::line);
  BOOST_TEST(std::get<1>(kick_flag_test2) == 2.0);
  command.set_position({3.0, 4.0, 5.0});
  {
    const auto [sp, sp_rot] = command.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::position>(sp);
    BOOST_TEST(std::get<0>(r1) == 3.0);
    BOOST_TEST(std::get<1>(r1) == 4.0);
    const auto r2 = std::get<ai_server::model::setpoint::angle>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 5.0);
  }
  command.set_velocity({6.0, 7.0, 8.0});
  {
    const auto [sp, sp_rot] = command.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::velocity>(sp);
    BOOST_TEST(std::get<0>(r1) == 6.0);
    BOOST_TEST(std::get<1>(r1) == 7.0);
    const auto r2 = std::get<ai_server::model::setpoint::velangular>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 8.0);
  }
}

BOOST_AUTO_TEST_CASE(new_if) {
  ai_server::model::command cmd{};

  // 初期値は速度0、角速度0
  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::velocity>(sp);
    BOOST_TEST(std::get<0>(r1) == 0);
    BOOST_TEST(std::get<1>(r1) == 0);

    const auto r2 = std::get<ai_server::model::setpoint::velangular>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 0);
  }

  cmd.set_position(123, 456);
  cmd.set_velanglar(789);

  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::position>(sp);
    BOOST_TEST(std::get<0>(r1) == 123);
    BOOST_TEST(std::get<1>(r1) == 456);

    const auto r2 = std::get<ai_server::model::setpoint::velangular>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 789);
  }

  cmd.set_velocity(321, 654);
  cmd.set_angle(987);

  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::velocity>(sp);
    BOOST_TEST(std::get<0>(r1) == 321);
    BOOST_TEST(std::get<1>(r1) == 654);

    const auto r2 = std::get<ai_server::model::setpoint::angle>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 987);
  }

  // 3引数のオーバーロード
  cmd.set_position(12, 34, 56);
  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::position>(sp);
    BOOST_TEST(std::get<0>(r1) == 12);
    BOOST_TEST(std::get<1>(r1) == 34);
    const auto r2 = std::get<ai_server::model::setpoint::angle>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 56);
  }
  cmd.set_velocity(21, 43, 65);
  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::velocity>(sp);
    BOOST_TEST(std::get<0>(r1) == 21);
    BOOST_TEST(std::get<1>(r1) == 43);
    const auto r2 = std::get<ai_server::model::setpoint::velangular>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 65);
  }

  cmd.set_velanglar(1000);

  // Vector2d を渡すケース
  cmd.set_position(Eigen::Vector2d(1.2, 3.4));
  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::position>(sp);
    BOOST_TEST(std::get<0>(r1) == 1.2);
    BOOST_TEST(std::get<1>(r1) == 3.4);
    const auto r2 = std::get<ai_server::model::setpoint::velangular>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 1000);
  }
  cmd.set_velocity(Eigen::Vector2d(2.1, 4.3));
  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::velocity>(sp);
    BOOST_TEST(std::get<0>(r1) == 2.1);
    BOOST_TEST(std::get<1>(r1) == 4.3);
    const auto r2 = std::get<ai_server::model::setpoint::velangular>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 1000);
  }

  // Vector3d を渡すケース
  cmd.set_position(Eigen::Vector3d(1.2, 3.4, 5.6));
  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::position>(sp);
    BOOST_TEST(std::get<0>(r1) == 1.2);
    BOOST_TEST(std::get<1>(r1) == 3.4);
    const auto r2 = std::get<ai_server::model::setpoint::angle>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 5.6);
  }
  cmd.set_velocity(Eigen::Vector3d(2.1, 4.3, 6.5));
  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::velocity>(sp);
    BOOST_TEST(std::get<0>(r1) == 2.1);
    BOOST_TEST(std::get<1>(r1) == 4.3);
    const auto r2 = std::get<ai_server::model::setpoint::velangular>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 6.5);
  }

  // Vector2d + double を渡すケース
  cmd.set_position(Eigen::Vector2d(1.2, 3.4), 5.6);
  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::position>(sp);
    BOOST_TEST(std::get<0>(r1) == 1.2);
    BOOST_TEST(std::get<1>(r1) == 3.4);
    const auto r2 = std::get<ai_server::model::setpoint::angle>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 5.6);
  }
  cmd.set_velocity(Eigen::Vector2d(2.1, 4.3), 6.5);
  {
    const auto [sp, sp_rot] = cmd.setpoint_pair();

    const auto r1 = std::get<ai_server::model::setpoint::velocity>(sp);
    BOOST_TEST(std::get<0>(r1) == 2.1);
    BOOST_TEST(std::get<1>(r1) == 4.3);
    const auto r2 = std::get<ai_server::model::setpoint::velangular>(sp_rot);
    BOOST_TEST(std::get<0>(r2) == 6.5);
  }
}

BOOST_AUTO_TEST_SUITE_END()
