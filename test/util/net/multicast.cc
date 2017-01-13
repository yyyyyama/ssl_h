#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE multicast_test

#include <chrono>
#include <future>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/util/net/multicast/receiver.h"
#include "ai_server/util/net/multicast/sender.h"

using namespace std::string_literals;
using namespace ai_server::util::net::multicast;

BOOST_AUTO_TEST_SUITE(multicast)

BOOST_AUTO_TEST_CASE(send_and_receive) {
  boost::asio::io_service io_service;

  // 受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.2, port = 10006
  receiver r(io_service, "0.0.0.0", "224.5.23.2", 10006);

  // 何か受信したらpromiseに値をセットする
  std::promise<std::string> p;
  r.on_receive([&p](const auto& buf, auto size) { p.set_value({buf.cbegin(), size}); });

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.2, port = 10006
  sender s(io_service, "224.5.23.2", 10006);

  // 受信を開始する
  std::thread t([&] { io_service.run(); });

  // 念の為少し待つ
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  // "Hello!" を送信
  s.send(boost::asio::buffer("Hello!"s));

  auto f = p.get_future();
  // 受信されるのを待ち, 結果が "Hello!" なのをチェックする
  BOOST_TEST(f.get() == "Hello!"s);

  // 受信の終了
  io_service.stop();
  t.join();
}

BOOST_AUTO_TEST_SUITE_END()
