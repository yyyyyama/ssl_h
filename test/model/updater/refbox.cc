#define BOOST_TEST_DYN_LINK

#include <limits>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/updater/refbox.h"
#include "ssl-protos/refbox/referee.pb.h"

BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::refbox::game_command)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::refbox::stage_name)

BOOST_AUTO_TEST_SUITE(updater_refbox)

BOOST_AUTO_TEST_CASE(normal) {
  ai_server::model::updater::refbox ru;

  {
    // デフォルトコンストラクが呼ばれたときに, 内部の値がちゃんと初期化されているか
    const auto r1 = ru.value();
    const auto r2 = ai_server::model::refbox{};

    BOOST_TEST(r1.packet_timestamp().time_since_epoch().count() ==
               r2.packet_timestamp().time_since_epoch().count());
    BOOST_TEST(r1.stage_time_left() == r2.stage_time_left());
    BOOST_TEST(r1.stage() == r2.stage());
    BOOST_TEST(r1.command() == r2.command());

    BOOST_TEST(r1.team_blue().name() == r2.team_blue().name());
    BOOST_TEST(r1.team_yellow().name() == r2.team_yellow().name());
  }

  ssl_protos::refbox::Referee referee{};
  {
    referee.set_packet_timestamp(1513688793680551);
    referee.set_stage(ssl_protos::refbox::Referee::Stage::Referee_Stage_NORMAL_FIRST_HALF_PRE);
    // stage_time_leftはoptionalなので送られてこない場合がある
    // referee.set_stage_time_left(2);
    referee.set_command_counter(3);
    referee.set_command(ssl_protos::refbox::Referee::Command::Referee_Command_HALT);
    referee.set_command_timestamp(4);

    ssl_protos::refbox::Referee_TeamInfo blue{};
    blue.set_name("blue");
    blue.set_score(10);
    blue.set_goalie(11);
    blue.set_red_cards(12);
    blue.set_yellow_cards(13);
    // yellow_card_timesは送られてこない可能性があるので,
    // そのテストとして値をセットしないこととする
    // blue.add_yellow_card_times(14);
    blue.set_timeouts(15);
    blue.set_timeout_time(16);
    referee.mutable_blue()->CopyFrom(blue);

    ssl_protos::refbox::Referee_TeamInfo yellow{};
    yellow.set_name("yellow");
    yellow.set_score(21);
    yellow.set_goalie(22);
    yellow.set_red_cards(23);
    yellow.set_yellow_cards(24);
    // yellow_card_timesは複数送られてくる場合がある
    yellow.add_yellow_card_times(25);
    yellow.add_yellow_card_times(26);
    yellow.add_yellow_card_times(27);
    yellow.set_timeouts(28);
    yellow.set_timeout_time(29);
    referee.mutable_yellow()->CopyFrom(yellow);
  }
  BOOST_REQUIRE_NO_THROW(ru.update(referee));

  {
    const auto r = ru.value();

    BOOST_TEST(r.packet_timestamp().time_since_epoch().count() ==
               ai_server::util::time_point_type{std::chrono::microseconds{1513688793680551}}
                   .time_since_epoch()
                   .count());
    // 送られてこなかったらmax()
    BOOST_TEST(r.stage_time_left() ==
               std::numeric_limits<decltype(r.stage_time_left())>::max());
    BOOST_TEST(r.stage() == ai_server::model::refbox::stage_name::normal_first_half_pre);
    BOOST_TEST(r.command() == ai_server::model::refbox::game_command::halt);

    const auto b = r.team_blue();
    BOOST_TEST(b.name() == referee.blue().name());
    BOOST_TEST(b.score() == referee.blue().score());
    BOOST_TEST(b.goalie() == referee.blue().goalie());
    BOOST_TEST(b.red_cards() == referee.blue().red_cards());
    BOOST_TEST(b.yellow_cards() == referee.blue().yellow_cards());
    // 送られてこなかったら0
    BOOST_TEST(b.yellow_card_times() == 0);
    BOOST_TEST(b.timeouts() == referee.blue().timeouts());
    BOOST_TEST(b.timeout_time() == referee.blue().timeout_time());

    const auto y = r.team_yellow();
    BOOST_TEST(y.name() == referee.yellow().name());
    BOOST_TEST(y.score() == referee.yellow().score());
    BOOST_TEST(y.goalie() == referee.yellow().goalie());
    BOOST_TEST(y.red_cards() == referee.yellow().red_cards());
    BOOST_TEST(y.yellow_cards() == referee.yellow().yellow_cards());
    // 複数送られてきたら先頭の値
    BOOST_TEST(y.yellow_card_times() == referee.yellow().yellow_card_times(0));
    BOOST_TEST(y.timeouts() == referee.yellow().timeouts());
    BOOST_TEST(y.timeout_time() == referee.yellow().timeout_time());
  }

  // stage_time_leftを設定してみる
  referee.set_stage_time_left(123);
  BOOST_REQUIRE_NO_THROW(ru.update(referee));
  BOOST_TEST(ru.value().stage_time_left() == 123);
}

BOOST_AUTO_TEST_SUITE_END()
