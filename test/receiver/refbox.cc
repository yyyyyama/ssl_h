#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE refbox_receiver_test

#include <chrono>
#include <future>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "../util/slot_testing_helper.h"

#include "ai_server/receiver/refbox.h"
#include "ai_server/util/net/multicast/sender.h"

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace ai_server;
using namespace ai_server::receiver;
using namespace ai_server::util::net::multicast;

BOOST_TEST_DONT_PRINT_LOG_VALUE(ssl_protos::refbox::Referee::Stage)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ssl_protos::refbox::Referee::Command)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::refbox::stage_name)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai_server::model::refbox::game_command)

BOOST_AUTO_TEST_SUITE(refbox_receiver)

BOOST_AUTO_TEST_CASE(send_and_receive, *boost::unit_test::timeout(30)) {
  // ダミーパケットの作成
  ssl_protos::refbox::Referee dummy_frame;

  ssl_protos::refbox::Referee_TeamInfo yellow;
  ssl_protos::refbox::Referee_TeamInfo blue;

  yellow.set_name("yellow");
  yellow.set_score(1);
  yellow.set_goalie(2);
  yellow.set_red_cards(3);
  yellow.set_yellow_cards(4);
  yellow.add_yellow_card_times(5);
  yellow.set_timeouts(6);
  yellow.set_timeout_time(7);
  blue.set_name("blue");
  blue.set_score(8);
  blue.set_goalie(9);
  blue.set_red_cards(10);
  blue.set_yellow_cards(11);
  blue.add_yellow_card_times(12);
  blue.set_timeouts(13);
  blue.set_timeout_time(14);

  dummy_frame.set_packet_timestamp(1);
  dummy_frame.set_stage(
      ssl_protos::refbox::Referee::Stage::Referee_Stage_NORMAL_FIRST_HALF_PRE);
  dummy_frame.set_stage_time_left(2);
  dummy_frame.set_command_counter(3);
  dummy_frame.set_command(ssl_protos::refbox::Referee::Command::Referee_Command_HALT);
  dummy_frame.set_command_timestamp(4);
  auto y = dummy_frame.mutable_yellow();
  y->CopyFrom(yellow);
  auto b = dummy_frame.mutable_blue();
  b->CopyFrom(blue);

  boost::asio::io_service io_service;

  // refbox受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.1, port = 10088
  refbox r(io_service, "0.0.0.0", "224.5.23.1", 10088);

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.1, port = 10088
  sender s(io_service, "224.5.23.1", 10088);

  // 受信を開始する
  std::thread t([&] { io_service.run(); });

  // 念の為少し待つ
  std::this_thread::sleep_for(50ms);

  {
    slot_testing_helper<ssl_protos::refbox::Referee> referee{&refbox::on_receive, r};

    // ダミーパケットを送信
    boost::asio::streambuf buf;
    std::ostream os(&buf);
    dummy_frame.SerializeToOstream(&os);
    s.send(buf.data());

    // 受信したデータを取得
    const auto f = std::get<0>(referee.result());

    // 受信したデータがダミーパケットと一致するか確認する

    BOOST_TEST(f.packet_timestamp() == dummy_frame.packet_timestamp());
    BOOST_TEST(f.stage() == dummy_frame.stage());
    BOOST_TEST(f.stage_time_left() == dummy_frame.stage_time_left());
    BOOST_TEST(f.command() == dummy_frame.command());
    BOOST_TEST(f.command_timestamp() == dummy_frame.command_timestamp());
    BOOST_TEST(f.yellow().name() == dummy_frame.yellow().name());
    BOOST_TEST(f.yellow().score() == dummy_frame.yellow().score());
    BOOST_TEST(f.yellow().goalie() == dummy_frame.yellow().goalie());
    BOOST_TEST(f.yellow().red_cards() == dummy_frame.yellow().red_cards());
    BOOST_TEST(f.yellow().yellow_cards() == dummy_frame.yellow().yellow_cards());
    BOOST_TEST(f.yellow().yellow_card_times(0) == dummy_frame.yellow().yellow_card_times(0));
    BOOST_TEST(f.yellow().timeouts() == dummy_frame.yellow().timeouts());
    BOOST_TEST(f.yellow().timeout_time() == dummy_frame.yellow().timeout_time());
    BOOST_TEST(f.blue().name() == dummy_frame.blue().name());
    BOOST_TEST(f.blue().score() == dummy_frame.blue().score());
    BOOST_TEST(f.blue().goalie() == dummy_frame.blue().goalie());
    BOOST_TEST(f.blue().red_cards() == dummy_frame.blue().red_cards());
    BOOST_TEST(f.blue().yellow_cards() == dummy_frame.blue().yellow_cards());
    BOOST_TEST(f.blue().yellow_card_times(0) == dummy_frame.blue().yellow_card_times(0));
    BOOST_TEST(f.blue().timeouts() == dummy_frame.blue().timeouts());
    BOOST_TEST(f.blue().timeout_time() == dummy_frame.blue().timeout_time());

    // model::refboxに変形できるかのテスト
    auto ref = r.refbox_data();

    BOOST_TEST(ref.packet_timestamp() == 1);
    BOOST_TEST(ref.stage() == model::refbox::stage_name::normal_first_half_pre);
    BOOST_TEST(ref.stage_time_left() == 2);
    BOOST_TEST(ref.command() == model::refbox::game_command::half);
    BOOST_TEST(ref.team_yellow().name() == "yellow");
    BOOST_TEST(ref.team_yellow().score() == 1);
    BOOST_TEST(ref.team_yellow().goalie() == 2);
    BOOST_TEST(ref.team_yellow().red_cards() == 3);
    BOOST_TEST(ref.team_yellow().yellow_cards() == 4);
    BOOST_TEST(ref.team_yellow().yellow_card_times() == 5);
    BOOST_TEST(ref.team_yellow().timeouts() == 6);
    BOOST_TEST(ref.team_yellow().timeout_time() == 7);
    BOOST_TEST(ref.team_blue().name() == "blue");
    BOOST_TEST(ref.team_blue().score() == 8);
    BOOST_TEST(ref.team_blue().goalie() == 9);
    BOOST_TEST(ref.team_blue().red_cards() == 10);
    BOOST_TEST(ref.team_blue().yellow_cards() == 11);
    BOOST_TEST(ref.team_blue().yellow_card_times() == 12);
    BOOST_TEST(ref.team_blue().timeouts() == 13);
    BOOST_TEST(ref.team_blue().timeout_time() == 14);
  }

  // 受信の終了
  io_service.stop();
  t.join();
}

BOOST_AUTO_TEST_CASE(non_protobuf_data, *boost::unit_test::timeout(30)) {
  boost::asio::io_service io_service;

  // refbox受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.4, port = 10080
  refbox r(io_service, "0.0.0.0", "224.5.23.4", 10080);

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.4, port = 10080
  sender s(io_service, "224.5.23.4", 10080);

  // 受信を開始する
  std::thread t([&] { io_service.run(); });

  // 念の為少し待つ
  std::this_thread::sleep_for(50ms);

  {
    slot_testing_helper<> referee{&refbox::on_error, r};

    // protobufじゃないデータを送信
    s.send("non protobuf data"s);

    // on_errorに設定したハンドラが呼ばれるまで待つ
    static_cast<void>(referee.result());
    BOOST_TEST(true);
  }

  // 受信の終了
  io_service.stop();
  t.join();
}

BOOST_AUTO_TEST_SUITE_END()