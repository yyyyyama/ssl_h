#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include "ai_server/model/refbox.h"
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::refbox::stage_name)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::refbox::game_command)

BOOST_AUTO_TEST_SUITE(refbox)

// team_info getter and initialization check
BOOST_AUTO_TEST_CASE(test01) {
  ai_server::model::refbox::team_info team_test{"t"};

  BOOST_TEST(team_test.name() == "t");
  BOOST_TEST(team_test.score() == 0);
  BOOST_TEST(team_test.goalie() == 0);
  BOOST_TEST(team_test.red_cards() == 0);
  BOOST_TEST(team_test.yellow_cards() == 0);
  BOOST_TEST(team_test.yellow_card_times() == 0);
  BOOST_TEST(team_test.timeouts() == 0);
  BOOST_TEST(team_test.timeout_time() == 0);
}

// team_info setter check
BOOST_AUTO_TEST_CASE(test02) {
  ai_server::model::refbox::team_info team_test{"t"};

  team_test.set_score(1);
  BOOST_TEST(team_test.score() == 1);
  team_test.set_goalie(2);
  BOOST_TEST(team_test.goalie() == 2);
  team_test.set_red_cards(3);
  BOOST_TEST(team_test.red_cards() == 3);
  team_test.set_yellow_cards(4);
  BOOST_TEST(team_test.yellow_cards() == 4);
  team_test.set_yellow_card_times(5);
  BOOST_TEST(team_test.yellow_card_times() == 5);
  team_test.set_timeouts(6);
  BOOST_TEST(team_test.timeouts() == 6);
  team_test.set_timeout_time(7);
  BOOST_TEST(team_test.timeout_time() == 7);
}

// refbox getter and initialization check
BOOST_AUTO_TEST_CASE(test03) {
  ai_server::model::refbox ref{};

  BOOST_TEST(ref.packet_timestamp() == 0);
  BOOST_TEST(ref.stage() == ai_server::model::refbox::stage_name::normal_first_half_pre);
  BOOST_TEST(ref.stage_time_left() == 0);
  BOOST_TEST(ref.command() == ai_server::model::refbox::game_command::halt);
  BOOST_TEST(ref.team_yellow().name() == "yellow");
  BOOST_TEST(ref.team_blue().name() == "blue");
}

// refbox setter check
BOOST_AUTO_TEST_CASE(test04) {
  ai_server::model::refbox ref{};

  ref.set_packet_timestamp(1);
  BOOST_TEST(ref.packet_timestamp() == 1);
  ref.set_stage(ai_server::model::refbox::stage_name::normal_first_half);
  BOOST_TEST(ref.stage() == ai_server::model::refbox::stage_name::normal_first_half);
  ref.set_stage_time_left(2);
  BOOST_TEST(ref.stage_time_left() == 2);
  ref.set_command(ai_server::model::refbox::game_command::stop);
  BOOST_TEST(ref.command() == ai_server::model::refbox::game_command::stop);
  ai_server::model::refbox::team_info test_yellow{"test_yellow"};
  ref.set_team_yellow(test_yellow);
  BOOST_TEST(ref.team_yellow().name() == test_yellow.name());
  ai_server::model::refbox::team_info test_blue{"test_blue"};
  ref.set_team_blue(test_blue);
  BOOST_TEST(ref.team_blue().name() == test_blue.name());
}

BOOST_AUTO_TEST_SUITE_END()
