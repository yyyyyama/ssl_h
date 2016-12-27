#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE refbox_test
#include <boost/test/unit_test.hpp>
#include "ai_server/model/refbox.h"
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::refbox::stage_name)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::refbox::game_command)

BOOST_AUTO_TEST_SUITE(refbox)

// getter and initialization check
BOOST_AUTO_TEST_CASE(test01) {
  ai_server::model::refbox ref{};

  BOOST_TEST(ref.packet_timestamp() == 0);
  BOOST_TEST(ref.stage() == ai_server::model::refbox::stage_name::normal_first_half_pre);
  BOOST_TEST(ref.stage_time_left() == 0);
  BOOST_TEST(ref.command() == ai_server::model::refbox::game_command::half);
  BOOST_TEST(ref.team_yellow().name() == "yellow");
  BOOST_TEST(ref.team_yellow().score() == 0);
  BOOST_TEST(ref.team_blue().name() == "blue");
}

// refbox setter check
BOOST_AUTO_TEST_CASE(test02) {
  ai_server::model::refbox ref{};

  ref.set_packet_timestamp(1);
  BOOST_TEST(ref.packet_timestamp() == 1);
  ref.set_stage(ai_server::model::refbox::stage_name::normal_first_half);
  BOOST_TEST(ref.stage() == ai_server::model::refbox::stage_name::normal_first_half);
  ref.set_stage_time_left(1);
  BOOST_TEST(ref.stage_time_left() == 1);
  ref.set_command(ai_server::model::refbox::game_command::stop);
  BOOST_TEST(ref.command() == ai_server::model::refbox::game_command::stop);
}

// team_info setter check
BOOST_AUTO_TEST_CASE(test03) {
  ai_server::model::refbox ref{};
  ai_server::model::refbox::team_info team_test{"t"};

  team_test.set_score(1);
  BOOST_TEST(team_test.score() == 1);
  ref.set_team_yellow(team_test);
  BOOST_TEST(ref.team_yellow().name() == team_test.name());
  ref.set_team_blue(team_test);
  BOOST_TEST(ref.team_blue().name() == team_test.name());
}

BOOST_AUTO_TEST_SUITE_END()
