#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE vision_receiver_test

#include <chrono>
#include <future>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/receiver/vision.h"
#include "ai_server/util/net/multicast/sender.h"

using namespace std::string_literals;
using namespace ai_server::receiver;
using namespace ai_server::util::net::multicast;

BOOST_AUTO_TEST_SUITE(vision_receiver)

BOOST_AUTO_TEST_CASE(send_and_receive) {
  // ダミーパケットの作成
  ssl_protos::vision::Frame dummy_frame;
  dummy_frame.set_frame_number(1);
  dummy_frame.set_t_capture(2.0);
  dummy_frame.set_t_sent(3.0);
  dummy_frame.set_camera_id(4);

  ssl_protos::vision::Packet dummy_packet;
  auto md = dummy_packet.mutable_detection();
  md->CopyFrom(dummy_frame);

  boost::asio::io_service io_service;

  // SSL-Vision受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.3, port = 10008
  vision v(io_service, "0.0.0.0", "224.5.23.3", 10008);

  // 何か受信したらpromiseに値をセットする
  std::promise<ssl_protos::vision::Packet> p;
  v.on_receive([&p](const auto& packet) { p.set_value(packet); });

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.3, port = 10008
  sender s(io_service, "224.5.23.3", 10008);

  // 受信を開始する
  std::thread t([&] { io_service.run(); });

  // 念の為少し待つ
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  // ダミーパケットを送信
  boost::asio::streambuf buf;
  std::ostream os(&buf);
  dummy_packet.SerializeToOstream(&os);
  s.send(buf.data());

  // データが受信されるのを待つ
  const auto f = p.get_future().get();

  // 受信したデータがダミーパケットと一致するか確認する
  BOOST_TEST(f.has_detection());
  BOOST_TEST(!f.has_geometry());

  const auto& d = f.detection();
  BOOST_TEST(d.frame_number() == dummy_frame.frame_number());
  BOOST_TEST(d.t_capture() == dummy_frame.t_capture());
  BOOST_TEST(d.t_sent() == dummy_frame.t_sent());
  BOOST_TEST(d.camera_id() == dummy_frame.camera_id());

  // 受信の終了
  io_service.stop();
  t.join();
}

BOOST_AUTO_TEST_SUITE_END()
