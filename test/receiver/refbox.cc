#define BOOST_TEST_DYN_LINK

#include <chrono>
#include <future>
#include <sstream>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "ssl-protos/gc_referee_message.pb.h"

#include "ai_server/logger/sink/ostream.h"
#include "ai_server/receiver/refbox.h"
#include "ai_server/util/net/multicast/sender.h"

#include "test_helpers/asio.h"
#include "test_helpers/signal_slot.h"

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace ai_server::logger;
using namespace ai_server::receiver;
using namespace ai_server::util::net::multicast;

BOOST_AUTO_TEST_SUITE(refbox_receiver)

BOOST_AUTO_TEST_CASE(receiver_error, *boost::unit_test::timeout(30)) {
  std::ostringstream s{};
  sink::ostream o{s, "nyan"};

  boost::asio::io_context ctx{};

  // 第2引数がマルチキャストアドレスではなく、
  // util::net::multicast::receiver 側でエラーが発生するケース
  refbox r{ctx, "0.0.0.0", "10.0.0.0", 10008};

  // まだエラーメッセージは出力されていない
  BOOST_TEST(s.str() == "");

  {
    slot_testing_helper wrapper{&refbox::on_error, r};

    auto t = run_io_context_in_new_thread(ctx);

    // on_error に登録したハンドラが呼ばれる
    BOOST_CHECK_NO_THROW(wrapper.result());

    // エラーメッセージが出力される
    BOOST_TEST(s.str() == "nyan\n");
  }
}

BOOST_AUTO_TEST_CASE(send_and_receive, *boost::unit_test::timeout(30)) {
  // ダミーパケットの作成
  ssl_protos::gc::Referee dummy_frame;

  ssl_protos::gc::Referee_TeamInfo yellow;
  ssl_protos::gc::Referee_TeamInfo blue;

  yellow.set_name("yellow");
  yellow.set_score(1);
  yellow.set_goalkeeper(2);
  yellow.set_red_cards(3);
  yellow.set_yellow_cards(4);
  yellow.add_yellow_card_times(5);
  yellow.set_timeouts(6);
  yellow.set_timeout_time(7);
  blue.set_name("blue");
  blue.set_score(8);
  blue.set_goalkeeper(9);
  blue.set_red_cards(10);
  blue.set_yellow_cards(11);
  blue.add_yellow_card_times(12);
  blue.set_timeouts(13);
  blue.set_timeout_time(14);

  dummy_frame.set_packet_timestamp(1);
  dummy_frame.set_stage(ssl_protos::gc::Referee::Stage::Referee_Stage_NORMAL_FIRST_HALF_PRE);
  dummy_frame.set_stage_time_left(2);
  dummy_frame.set_command_counter(3);
  dummy_frame.set_command(ssl_protos::gc::Referee::Command::Referee_Command_HALT);
  dummy_frame.set_command_timestamp(4);
  auto y = dummy_frame.mutable_yellow();
  y->CopyFrom(yellow);
  auto b = dummy_frame.mutable_blue();
  b->CopyFrom(blue);

  boost::asio::io_context ctx{};

  // refbox受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.1, port = 10088
  refbox r{ctx, "0.0.0.0", "224.5.23.1", 10088};

  // 初期値は0
  BOOST_TEST(r.total_messages() == 0);
  BOOST_TEST(r.messages_per_second() == 0);
  BOOST_TEST(r.parse_error() == 0);
  BOOST_TEST((r.last_updated() == std::chrono::system_clock::time_point{}));

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.1, port = 10088
  sender s{ctx, "224.5.23.1", 10088};

  // 受信を開始する
  auto t = run_io_context_in_new_thread(ctx);

  {
    slot_testing_helper<ssl_protos::gc::Referee> referee{&refbox::on_receive, r};

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
    BOOST_TEST(f.yellow().goalkeeper() == dummy_frame.yellow().goalkeeper());
    BOOST_TEST(f.yellow().red_cards() == dummy_frame.yellow().red_cards());
    BOOST_TEST(f.yellow().yellow_cards() == dummy_frame.yellow().yellow_cards());
    BOOST_TEST(f.yellow().yellow_card_times(0) == dummy_frame.yellow().yellow_card_times(0));
    BOOST_TEST(f.yellow().timeouts() == dummy_frame.yellow().timeouts());
    BOOST_TEST(f.yellow().timeout_time() == dummy_frame.yellow().timeout_time());
    BOOST_TEST(f.blue().name() == dummy_frame.blue().name());
    BOOST_TEST(f.blue().score() == dummy_frame.blue().score());
    BOOST_TEST(f.blue().goalkeeper() == dummy_frame.blue().goalkeeper());
    BOOST_TEST(f.blue().red_cards() == dummy_frame.blue().red_cards());
    BOOST_TEST(f.blue().yellow_cards() == dummy_frame.blue().yellow_cards());
    BOOST_TEST(f.blue().yellow_card_times(0) == dummy_frame.blue().yellow_card_times(0));
    BOOST_TEST(f.blue().timeouts() == dummy_frame.blue().timeouts());
    BOOST_TEST(f.blue().timeout_time() == dummy_frame.blue().timeout_time());

    // 情報が更新されている
    BOOST_TEST(r.total_messages() == 1);
    BOOST_TEST(r.parse_error() == 0);
  }

  // 前回の1秒間に受信したメッセージは1
  std::this_thread::sleep_for(1s);
  BOOST_TEST(r.messages_per_second() == 1);
}

BOOST_AUTO_TEST_CASE(non_protobuf_data, *boost::unit_test::timeout(30)) {
  std::ostringstream ss{};
  sink::ostream o{ss, "{message}"};

  boost::asio::io_context ctx{};

  // refbox受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.4, port = 10080
  refbox r{ctx, "0.0.0.0", "224.5.23.4", 10080};

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.4, port = 10080
  sender s{ctx, "224.5.23.4", 10080};

  // 受信を開始する
  auto t = run_io_context_in_new_thread(ctx);

  {
    slot_testing_helper<> referee{&refbox::on_error, r};

    // protobufじゃないデータを送信
    s.send("non protobuf data"s);

    // on_errorに設定したハンドラが呼ばれるまで待つ
    static_cast<void>(referee.result());
    BOOST_TEST(true);

    // 情報が更新されている
    BOOST_TEST(r.total_messages() == 1);
    BOOST_TEST(r.parse_error() == 1);

    // 警告が出力されている
    BOOST_TEST(ss.str() == "failed to parse message 1\n");
  }

  // 前回の1秒間に受信したメッセージは1
  std::this_thread::sleep_for(1s);
  BOOST_TEST(r.messages_per_second() == 1);
}

BOOST_AUTO_TEST_SUITE_END()
