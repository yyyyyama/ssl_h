#define BOOST_TEST_DYN_LINK

#include <limits>
#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/updater/refbox.h"
#include "ai_server/util/math/affine.h"
#include "ssl-protos/gc_referee_message.pb.h"

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

  ssl_protos::gc::Referee referee{};
  {
    referee.set_packet_timestamp(1513688793680551);
    referee.set_stage(ssl_protos::gc::Referee::Stage::Referee_Stage_NORMAL_FIRST_HALF_PRE);
    // stage_time_leftはoptionalなので送られてこない場合がある
    // referee.set_stage_time_left(2);
    referee.set_command_counter(3);
    referee.set_command(ssl_protos::gc::Referee::Command::Referee_Command_HALT);
    referee.set_command_timestamp(4);

    ssl_protos::gc::Referee_TeamInfo blue{};
    blue.set_name("blue");
    blue.set_score(10);
    blue.set_goalkeeper(11);
    blue.set_red_cards(12);
    blue.set_yellow_cards(13);
    // yellow_card_timesは送られてこない可能性があるので,
    // そのテストとして値をセットしないこととする
    // blue.add_yellow_card_times(14);
    blue.set_timeouts(15);
    blue.set_timeout_time(16);
    blue.set_max_allowed_bots(17);
    referee.mutable_blue()->CopyFrom(blue);

    ssl_protos::gc::Referee_TeamInfo yellow{};
    yellow.set_name("yellow");
    yellow.set_score(21);
    yellow.set_goalkeeper(22);
    yellow.set_red_cards(23);
    yellow.set_yellow_cards(24);
    // yellow_card_timesは複数送られてくる場合がある
    yellow.add_yellow_card_times(25);
    yellow.add_yellow_card_times(26);
    yellow.add_yellow_card_times(27);
    yellow.set_timeouts(28);
    yellow.set_timeout_time(29);
    yellow.set_max_allowed_bots(30);
    referee.mutable_yellow()->CopyFrom(yellow);
  }
  BOOST_REQUIRE_NO_THROW(ru.update(referee));

  {
    const auto r = ru.value();

    BOOST_TEST(
        r.packet_timestamp().time_since_epoch().count() ==
        std::chrono::system_clock::time_point{std::chrono::microseconds{1513688793680551}}
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
    BOOST_TEST(b.goalie() == referee.blue().goalkeeper());
    BOOST_TEST(b.red_cards() == referee.blue().red_cards());
    BOOST_TEST(b.yellow_cards() == referee.blue().yellow_cards());
    // 送られてこなかったら0
    BOOST_TEST(b.yellow_card_times() == 0);
    BOOST_TEST(b.timeouts() == referee.blue().timeouts());
    BOOST_TEST(b.timeout_time() == referee.blue().timeout_time());
    BOOST_TEST(b.max_allowed_bots() == referee.blue().max_allowed_bots());

    const auto y = r.team_yellow();
    BOOST_TEST(y.name() == referee.yellow().name());
    BOOST_TEST(y.score() == referee.yellow().score());
    BOOST_TEST(y.goalie() == referee.yellow().goalkeeper());
    BOOST_TEST(y.red_cards() == referee.yellow().red_cards());
    BOOST_TEST(y.yellow_cards() == referee.yellow().yellow_cards());
    // 複数送られてきたら先頭の値
    BOOST_TEST(y.yellow_card_times() == referee.yellow().yellow_card_times(0));
    BOOST_TEST(y.timeouts() == referee.yellow().timeouts());
    BOOST_TEST(y.timeout_time() == referee.yellow().timeout_time());
    BOOST_TEST(y.max_allowed_bots() == referee.yellow().max_allowed_bots());
  }

  // stage_time_leftを設定してみる
  referee.set_stage_time_left(123);
  BOOST_REQUIRE_NO_THROW(ru.update(referee));
  BOOST_TEST(ru.value().stage_time_left() == 123);
}

BOOST_AUTO_TEST_CASE(abp, *boost::unit_test::tolerance(0.0000001)) {
  ssl_protos::gc::Referee referee{};
  {
    referee.set_packet_timestamp(1513688793680551);
    referee.set_stage(ssl_protos::gc::Referee::Stage::Referee_Stage_NORMAL_FIRST_HALF_PRE);
    referee.set_command_counter(3);
    referee.set_command(ssl_protos::gc::Referee::Command::Referee_Command_BALL_PLACEMENT_BLUE);
    referee.set_command_timestamp(4);

    ssl_protos::gc::Referee_TeamInfo blue{};
    blue.set_name("blue");
    blue.set_score(10);
    blue.set_goalkeeper(11);
    blue.set_red_cards(12);
    blue.set_yellow_cards(13);
    blue.set_timeouts(15);
    blue.set_timeout_time(16);
    blue.set_max_allowed_bots(17);
    referee.mutable_blue()->CopyFrom(blue);

    ssl_protos::gc::Referee_TeamInfo yellow{};
    yellow.set_name("yellow");
    yellow.set_score(21);
    yellow.set_goalkeeper(22);
    yellow.set_red_cards(23);
    yellow.set_yellow_cards(24);
    yellow.set_timeouts(28);
    yellow.set_timeout_time(29);
    yellow.set_max_allowed_bots(30);
    referee.mutable_yellow()->CopyFrom(yellow);

    ssl_protos::gc::Referee_Point point{};
    point.set_x(100);
    point.set_y(200);
    referee.mutable_designated_position()->CopyFrom(point);
  }

  {
    // 座標変換なし
    ai_server::model::updater::refbox ru{};
    ru.update(referee);

    const auto& r = ru.value();
    BOOST_TEST(r.ball_placement_position().x() == 100.0);
    BOOST_TEST(r.ball_placement_position().y() == 200.0);
  }

  {
    // 座標変換あり
    using namespace boost::math::double_constants;
    const auto mat = ai_server::util::math::make_transformation_matrix(10.0, 20.0, half_pi);

    ai_server::model::updater::refbox ru{};
    ru.set_transformation_matrix(mat);
    ru.update(referee);

    const auto& r = ru.value();
    BOOST_TEST(r.ball_placement_position().x() == -200.0 + 10);
    BOOST_TEST(r.ball_placement_position().y() == 100.0 + 20);
  }
}

BOOST_AUTO_TEST_CASE(bot_substitution) {
  ssl_protos::gc::Referee referee{};
  {
    referee.set_packet_timestamp(1513688793680551);
    referee.set_stage(ssl_protos::gc::Referee::Stage::Referee_Stage_NORMAL_FIRST_HALF_PRE);
    referee.set_command(ssl_protos::gc::Referee::Command::Referee_Command_HALT);
    referee.set_command_counter(3);
    referee.set_command_timestamp(4);

    auto blue = referee.mutable_blue();
    blue->set_name("blue");
    blue->set_score(10);
    blue->set_goalkeeper(11);
    blue->set_red_cards(12);
    blue->set_yellow_cards(13);
    blue->set_timeouts(15);
    blue->set_timeout_time(16);
    blue->set_max_allowed_bots(17);

    auto yellow = referee.mutable_yellow();
    yellow->set_name("yellow");
    yellow->set_score(21);
    yellow->set_goalkeeper(22);
    yellow->set_red_cards(23);
    yellow->set_yellow_cards(24);
    yellow->set_timeouts(28);
    yellow->set_timeout_time(29);
    yellow->set_max_allowed_bots(30);
  }

  ai_server::model::updater::refbox ru{};

  // GameEvent なしのときは std::nullopt
  {
    referee.clear_game_events();
    ru.update(referee);

    const auto r = ru.value();
    BOOST_TEST(!r.bot_substitution_by_team().has_value());
  }

  // game_events に YELLOW の BotSubstitution イベントだけがあったとき
  {
    referee.clear_game_events();

    auto e = referee.add_game_events()->mutable_bot_substitution();
    e->set_by_team(ssl_protos::gc::YELLOW);

    ru.update(referee);

    const auto r = ru.value();
    BOOST_TEST((r.bot_substitution_by_team() == ai_server::model::team_color::yellow));
  }

  // game_events に BLUE の BotSubstitution イベントだけがあったとき
  {
    referee.clear_game_events();

    auto e = referee.add_game_events()->mutable_bot_substitution();
    e->set_by_team(ssl_protos::gc::BLUE);

    ru.update(referee);

    const auto r = ru.value();
    BOOST_TEST((r.bot_substitution_by_team() == ai_server::model::team_color::blue));
  }

  // game_events に BotSubstitution 以外のイベントだけがあったとき
  {
    referee.clear_game_events();

    auto e = referee.add_game_events()->mutable_prepared();
    e->set_time_taken(1.23);

    ru.update(referee);

    const auto r = ru.value();
    BOOST_TEST(!r.bot_substitution_by_team().has_value());
  }

  // game_events に複数のイベントが含まれているとき (1)
  {
    referee.clear_game_events();

    auto e1 = referee.add_game_events()->mutable_prepared();
    e1->set_time_taken(1.23);

    auto e2 = referee.add_game_events()->mutable_no_progress_in_game();
    e2->set_time(4.56);

    auto e3 = referee.add_game_events()->mutable_bot_substitution();
    e3->set_by_team(ssl_protos::gc::BLUE);

    ru.update(referee);

    const auto r = ru.value();
    BOOST_TEST((r.bot_substitution_by_team() == ai_server::model::team_color::blue));
  }

  // game_events に複数のイベントが含まれているとき (2)
  {
    referee.clear_game_events();

    auto e1 = referee.add_game_events()->mutable_prepared();
    e1->set_time_taken(1.23);

    auto e2 = referee.add_game_events()->mutable_no_progress_in_game();
    e2->set_time(4.56);

    ru.update(referee);

    const auto r = ru.value();
    BOOST_TEST(!r.bot_substitution_by_team().has_value());
  }

  // game_events に複数のイベントが含まれているとき (3)
  {
    referee.clear_game_events();

    auto e1 = referee.add_game_events()->mutable_prepared();
    e1->set_time_taken(1.23);

    auto e2 = referee.add_game_events()->mutable_bot_substitution();
    e2->set_by_team(ssl_protos::gc::YELLOW);

    auto e3 = referee.add_game_events()->mutable_no_progress_in_game();
    e3->set_time(4.56);

    ru.update(referee);

    const auto r = ru.value();
    BOOST_TEST((r.bot_substitution_by_team() == ai_server::model::team_color::yellow));
  }
}

BOOST_AUTO_TEST_SUITE_END()
