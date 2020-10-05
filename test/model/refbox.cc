#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include "ai_server/model/refbox.h"
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::refbox::stage_name)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::refbox::game_command)

BOOST_AUTO_TEST_SUITE(refbox)

// team_info getter and initialization check
BOOST_AUTO_TEST_CASE(test01) {
  ai_server::model::team_info team_test{"t"};

  BOOST_TEST(team_test.name() == "t");
  BOOST_TEST(team_test.score() == 0);
  BOOST_TEST(team_test.goalie() == 0);
  BOOST_TEST(team_test.red_cards() == 0);
  BOOST_TEST(team_test.yellow_cards() == 0);
  BOOST_TEST(team_test.yellow_card_times() == 0);
  BOOST_TEST(team_test.timeouts() == 0);
  BOOST_TEST(team_test.timeout_time() == 0);
  BOOST_TEST(team_test.max_allowed_bots() == 11);
}

// team_info setter check
BOOST_AUTO_TEST_CASE(test02) {
  ai_server::model::team_info team_test{"t"};

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
  team_test.set_max_allowed_bots(8);
  BOOST_TEST(team_test.max_allowed_bots() == 8);
}

// refbox getter and initialization check
BOOST_AUTO_TEST_CASE(test03) {
  ai_server::model::refbox ref{};

  BOOST_TEST(ref.packet_timestamp().time_since_epoch().count() == 0);
  BOOST_TEST(ref.stage() == ai_server::model::refbox::stage_name::normal_first_half_pre);
  BOOST_TEST(ref.stage_time_left() == 0);
  BOOST_TEST(ref.command() == ai_server::model::refbox::game_command::halt);
  BOOST_TEST(ref.team_yellow().name() == "yellow");
  BOOST_TEST(ref.team_blue().name() == "blue");
  BOOST_TEST(ref.ball_placement_position().x() == 0.0);
  BOOST_TEST(ref.ball_placement_position().y() == 0.0);
}

// refbox setter check
BOOST_AUTO_TEST_CASE(test04) {
  ai_server::model::refbox ref{};

  constexpr auto dummy_time =
      std::chrono::system_clock::time_point{std::chrono::microseconds{1513688793680551}};
  ref.set_packet_timestamp(dummy_time);
  BOOST_TEST(ref.packet_timestamp().time_since_epoch().count() ==
             dummy_time.time_since_epoch().count());
  ref.set_stage(ai_server::model::refbox::stage_name::normal_first_half);
  BOOST_TEST(ref.stage() == ai_server::model::refbox::stage_name::normal_first_half);
  ref.set_stage_time_left(2);
  BOOST_TEST(ref.stage_time_left() == 2);
  ref.set_command(ai_server::model::refbox::game_command::stop);
  BOOST_TEST(ref.command() == ai_server::model::refbox::game_command::stop);
  ai_server::model::team_info test_yellow{"test_yellow"};
  ref.set_team_yellow(test_yellow);
  BOOST_TEST(ref.team_yellow().name() == test_yellow.name());
  ai_server::model::team_info test_blue{"test_blue"};
  ref.set_team_blue(test_blue);
  BOOST_TEST(ref.team_blue().name() == test_blue.name());

  ref.set_ball_placement_position({1.23, 4.56});
  BOOST_TEST(ref.ball_placement_position().x() == 1.23);
  BOOST_TEST(ref.ball_placement_position().y() == 4.56);
}

BOOST_AUTO_TEST_CASE(bot_substitution) {
  ai_server::model::refbox ref{};

  // bot_substitution getter and initialization check
  BOOST_TEST(!ref.bot_substitution_by_team().has_value());

  // bot_substitution setter check
  ref.set_bot_substitution_by_team(ai_server::model::team_color::yellow);
  BOOST_TEST((ref.bot_substitution_by_team() == ai_server::model::team_color::yellow));
  ref.set_bot_substitution_by_team(ai_server::model::team_color::blue);
  BOOST_TEST((ref.bot_substitution_by_team() == ai_server::model::team_color::blue));
  ref.set_bot_substitution_by_team(std::nullopt);
  BOOST_TEST(!ref.bot_substitution_by_team().has_value());
}

BOOST_AUTO_TEST_CASE(team_info_heplers) {
  ai_server::model::refbox ref{};
  ref.set_team_yellow({"yellow team name"});
  ref.set_team_blue({"blue team name"});

  // our_team_info(ref, yellow) で黄チームの情報が取得できる
  BOOST_TEST(
      ai_server::model::our_team_info(ref, ai_server::model::team_color::yellow).name() ==
      "yellow team name");
  // our_team_info(ref, bluw) で青チームの情報が取得できる
  BOOST_TEST(ai_server::model::our_team_info(ref, ai_server::model::team_color::blue).name() ==
             "blue team name");

  // enemy_team_info(ref, yellow) で青チームの情報が取得できる
  BOOST_TEST(
      ai_server::model::enemy_team_info(ref, ai_server::model::team_color::yellow).name() ==
      "blue team name");
  // enemy_team_info(ref, bluw) で黄チームの情報が取得できる
  BOOST_TEST(
      ai_server::model::enemy_team_info(ref, ai_server::model::team_color::blue).name() ==
      "yellow team name");
}

BOOST_AUTO_TEST_SUITE_END()
