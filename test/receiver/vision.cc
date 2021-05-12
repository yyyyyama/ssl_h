#define BOOST_TEST_DYN_LINK

#include <chrono>
#include <future>
#include <sstream>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "ssl-protos/vision_wrapper.pb.h"

#include "ai_server/logger/sink/ostream.h"
#include "ai_server/receiver/vision.h"
#include "ai_server/util/net/multicast/sender.h"

#include "test_helpers/asio.h"
#include "test_helpers/signal_slot.h"

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace ai_server::logger;
using namespace ai_server::receiver;
using namespace ai_server::util::net::multicast;

BOOST_AUTO_TEST_SUITE(vision_receiver)

BOOST_AUTO_TEST_CASE(receiver_error, *boost::unit_test::timeout(30)) {
  std::ostringstream s{};
  sink::ostream o{s, "nyan"};

  boost::asio::io_context ctx{};

  // 第2引数がマルチキャストアドレスではなく、
  // util::net::multicast::receiver 側でエラーが発生するケース
  vision v{ctx, "0.0.0.0", "10.0.0.0", 10008};

  // まだエラーメッセージは出力されていない
  BOOST_TEST(s.str() == "");

  {
    slot_testing_helper wrapper{&vision::on_error, v};

    auto t = run_io_context_in_new_thread(ctx);

    // on_error に登録したハンドラが呼ばれる
    BOOST_CHECK_NO_THROW(wrapper.result());

    // エラーメッセージが出力される
    BOOST_TEST(s.str() == "nyan\n");
  }
}

BOOST_AUTO_TEST_CASE(send_and_receive, *boost::unit_test::timeout(30)) {
  // ダミーパケットの作成
  ssl_protos::vision::Frame dummy_frame;
  dummy_frame.set_frame_number(1);
  dummy_frame.set_t_capture(2.0);
  dummy_frame.set_t_sent(3.0);
  dummy_frame.set_camera_id(4);

  ssl_protos::vision::Packet dummy_packet;
  auto md = dummy_packet.mutable_detection();
  md->CopyFrom(dummy_frame);

  boost::asio::io_context ctx{};

  // SSL-Vision受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.3, port = 10008
  vision v{ctx, "0.0.0.0", "224.5.23.3", 10008};

  // 初期値は0
  BOOST_TEST(v.total_messages() == 0);
  BOOST_TEST(v.messages_per_second() == 0);
  BOOST_TEST(v.parse_error() == 0);
  BOOST_TEST((v.last_updated() == std::chrono::system_clock::time_point{}));

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.3, port = 10008
  sender s{ctx, "224.5.23.3", 10008};

  // 受信を開始する
  auto t = run_io_context_in_new_thread(ctx);

  {
    slot_testing_helper<ssl_protos::vision::Packet> wrapper{&vision::on_receive, v};

    // ダミーパケットを送信
    boost::asio::streambuf buf;
    std::ostream os(&buf);
    dummy_packet.SerializeToOstream(&os);
    s.send(buf.data());

    // 受信したデータを取得
    const auto f = std::get<0>(wrapper.result());

    // 受信したデータがダミーパケットと一致するか確認する
    BOOST_TEST(f.has_detection());
    BOOST_TEST(!f.has_geometry());

    const auto& d = f.detection();
    BOOST_TEST(d.frame_number() == dummy_frame.frame_number());
    BOOST_TEST(d.camera_id() == dummy_frame.camera_id());

    // 情報が更新されている
    BOOST_TEST(v.total_messages() == 1);
    BOOST_TEST(v.parse_error() == 0);
  }

  // 前回の1秒間に受信したメッセージは1
  std::this_thread::sleep_for(1s);
  BOOST_TEST(v.messages_per_second() == 1);
}

BOOST_AUTO_TEST_CASE(timestamp_adjustment,
                     *boost::unit_test::timeout(30) * boost::unit_test::tolerance(0.001)) {
  auto current_time = [] {
    constexpr auto den = std::chrono::system_clock::duration::period::den;
    constexpr auto num = std::chrono::system_clock::duration::period::num;

    // time を Vision で使われる形式 (double で単位が秒) に変換
    const auto time = std::chrono::system_clock::now();
    const auto te   = time.time_since_epoch();
    return static_cast<double>(te.count() * num) / den;
  };

  boost::asio::io_context ctx{};

  // SSL-Vision受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.3, port = 10009
  vision v{ctx, "0.0.0.0", "224.5.23.3", 10009};

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.3, port = 10009
  sender s{ctx, "224.5.23.3", 10009};

  // 受信を開始する
  auto t = run_io_context_in_new_thread(ctx);

  for (auto i = 0u; i < 5; ++i) {
    for (auto cam_id = 0u; cam_id < 5; ++cam_id) {
      slot_testing_helper<ssl_protos::vision::Packet> wrapper{&vision::on_receive, v};

      const auto tt = current_time();

      // cam_id 秒の時差があるとする
      ssl_protos::vision::Packet p{};
      {
        auto md = p.mutable_detection();
        md->set_frame_number(1);
        md->set_camera_id(cam_id);

        md->set_t_sent(tt + cam_id);
        md->set_t_capture(tt - 0.5 + cam_id);
      }

      boost::asio::streambuf buf{};
      std::ostream os(&buf);
      p.SerializeToOstream(&os);
      s.send(buf.data());

      // 受信したデータを取得
      const auto f = std::get<0>(wrapper.result());

      BOOST_TEST(f.has_detection());
      BOOST_TEST(!f.has_geometry());

      const auto& d = f.detection();
      BOOST_TEST(d.camera_id() == cam_id);
      // 時差が修正されている
      BOOST_TEST(d.t_sent() == tt);
      BOOST_TEST(d.t_capture() == tt - 0.5);
    }

    std::this_thread::sleep_for(50ms);
  }
}

BOOST_AUTO_TEST_CASE(non_protobuf_data, *boost::unit_test::timeout(30)) {
  std::ostringstream ss{};
  sink::ostream o{ss, "{message}"};

  boost::asio::io_context ctx{};

  // SSL-Vision受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.4, port = 10010
  vision v{ctx, "0.0.0.0", "224.5.23.4", 10010};

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.4, port = 10010
  sender s{ctx, "224.5.23.4", 10010};

  // 受信を開始する
  auto t = run_io_context_in_new_thread(ctx);

  {
    slot_testing_helper<> wrapper{&vision::on_error, v};

    // protobufじゃないデータを送信
    s.send("non protobuf data"s);

    // on_errorに設定したハンドラが呼ばれるまで待つ
    static_cast<void>(wrapper.result());
    BOOST_TEST(true);

    // 情報が更新されている
    BOOST_TEST(v.total_messages() == 1);
    BOOST_TEST(v.parse_error() == 1);

    // 警告が出力されている
    BOOST_TEST(ss.str() == "failed to parse message 1\n");
  }

  // 前回の1秒間に受信したメッセージは1
  std::this_thread::sleep_for(1s);
  BOOST_TEST(v.messages_per_second() == 1);
}

BOOST_AUTO_TEST_SUITE_END()
