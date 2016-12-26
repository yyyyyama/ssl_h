#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE refbox_test
#include <boost/test/unit_test.hpp>
#include "ai_server/model/refbox.h"
#include <memory>

BOOST_AUTO_TEST_SUITE(refbox)

// getter and initialization check
BOOST_AUTO_TEST_CASE(test01) {
  ai_server::model::refbox ref{};

  BOOST_TEST(ref.packet_timestamp() == 0);
  switch (ref.stage()) {
    case (ai_server::model::refbox::stage_name::normal_first_half_pre):
      BOOST_TEST(1);
      break;
    default:
      BOOST_TEST(0);
      break;
  }
  BOOST_TEST(ref.stage_time_left() == 0);
  switch (ref.command()) {
    case (ai_server::model::refbox::game_command::half):
      BOOST_TEST(1);
      break;
    default:
      BOOST_TEST(0);
      break;
  }
  BOOST_TEST(ref.team_yellow().name() == "yellow");
  BOOST_TEST(ref.team_yellow().score() == 0);
  BOOST_TEST(ref.team_yellow().goalie() == 0);
  BOOST_TEST(ref.team_yellow().red_cards() == 0);
  BOOST_TEST(ref.team_yellow().yellow_cards() == 0);
  BOOST_TEST(ref.team_yellow().yellow_card_times() == 0);
  BOOST_TEST(ref.team_yellow().timeouts() == 0);
  BOOST_TEST(ref.team_yellow().timeout_time() == 0);
  BOOST_TEST(ref.team_blue().name() == "blue");
}

// setter check
BOOST_AUTO_TEST_CASE(test02) {
  ai_server::model::refbox ref{};
  ai_server::model::refbox::team_info team_test{"t"};

  ref.set_packet_timestamp(1);
  BOOST_TEST(ref.packet_timestamp() == 1);

  ref.set_stage(ai_server::model::refbox::stage_name::normal_first_half);
  switch (ref.stage()) {
    case (ai_server::model::refbox::stage_name::normal_first_half):
      BOOST_TEST(1);
      break;
    default:
      BOOST_TEST(0);
      break;
  }

  ref.set_stage_time_left(1);
  BOOST_TEST(ref.stage_time_left() == 1);

  ref.set_command(ai_server::model::refbox::game_command::stop);
  switch (ref.command()) {
    case (ai_server::model::refbox::game_command::stop):
      BOOST_TEST(1);
      break;
    default:
      BOOST_TEST(0);
      break;
  }

  team_test.set_score(1);
  BOOST_TEST(team_test.score() == 1);
  team_test.set_goalie(1);
  BOOST_TEST(team_test.goalie() == 1);
  team_test.set_red_cards(1);
  BOOST_TEST(team_test.red_cards() == 1);
  team_test.set_yellow_cards(1);
  BOOST_TEST(team_test.yellow_cards() == 1);
  team_test.set_yellow_card_times(1);
  BOOST_TEST(team_test.yellow_card_times() == 1);
  team_test.set_timeouts(1);
  BOOST_TEST(team_test.timeouts() == 1);
  team_test.set_timeout_time(1);
  BOOST_TEST(team_test.timeout_time() == 1);

  ref.set_team_yellow(team_test);
  BOOST_TEST(ref.team_yellow().name() == team_test.name());
  ref.set_team_blue(team_test);
  BOOST_TEST(ref.team_blue().name() == team_test.name());
}

BOOST_AUTO_TEST_SUITE_END()
