#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE multicast_test

#include <chrono>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "../slot_testing_helper.h"

#include "ai_server/util/net/multicast/receiver.h"
#include "ai_server/util/net/multicast/sender.h"

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace ai_server::util::net::multicast;

using receiver_wrapper = slot_testing_helper<receiver::buffer_t, std::size_t>;

BOOST_AUTO_TEST_SUITE(multicast, *boost::unit_test::timeout(15))

BOOST_AUTO_TEST_CASE(send_and_receive) {
  boost::asio::io_service io_service;

  // 受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.2, port = 10006
  receiver r(io_service, "0.0.0.0", "224.5.23.2", 10006);

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.2, port = 10006
  sender s(io_service, "224.5.23.2", 10006);

  // 受信を開始する
  std::thread t([&] { io_service.run(); });

  // 念の為少し待つ
  std::this_thread::sleep_for(50ms);

  {
    receiver_wrapper wrapper{&receiver::on_receive, r};

    // std::string("Hello!") を送信
    const auto s1 = "Hello!"s;
    s.send(boost::asio::buffer(s1));

    // 受信したデータを取得
    const auto result = wrapper.result();

    // 受信したデータが送信したものと一致するか
    BOOST_TEST(std::get<1>(result) == s1.length());
    const auto s2 = std::string(std::cbegin(std::get<0>(result)), std::get<1>(result));
    BOOST_TEST(s2 == s1);
  }

  {
    receiver_wrapper wrapper{&receiver::on_receive, r};

    // receiverのバッファサイズ分(4096byte)の文字列を送信
    const auto s1 = std::string(receiver::buffer_size / 4, 'a') +
                    std::string(receiver::buffer_size / 4, 'b') +
                    std::string(receiver::buffer_size / 4, 'c') +
                    std::string(receiver::buffer_size / 4, 'd');
    s.send(boost::asio::buffer(s1));

    // 受信したデータを取得
    const auto result = wrapper.result();

    // 受信したデータが送信したものと一致するか
    BOOST_TEST(std::get<1>(result) == s1.length());
    const auto s2 = std::string(std::cbegin(std::get<0>(result)), std::get<1>(result));
    BOOST_TEST(s2 == s1);
  }

  // 乱数生成機を初期化
  std::mt19937 mt{std::random_device{}()};

  for (auto i = 0u; i < 1024; ++i) {
    receiver_wrapper wrapper{&receiver::on_receive, r};

    using value_type = typename receiver::buffer_t::value_type;

    // ランダムな長さ[1, 4096]のランダムなデータの入った配列を生成
    const auto size = std::uniform_int_distribution<std::size_t>(1, receiver::buffer_size)(mt);
    const auto v1   = [&mt, size] {
      std::vector<value_type> v(size);
      std::uniform_int_distribution<value_type> d(std::numeric_limits<value_type>::min(),
                                                  std::numeric_limits<value_type>::max());
      std::generate(v.begin(), v.end(), [&mt, &d] { return d(mt); });
      return v;
    }();
    s.send(boost::asio::buffer(v1));

    // 受信したデータを取得
    const auto result = wrapper.result();

    // 受信したデータが送信したものと一致するか
    BOOST_TEST(std::get<1>(result) == v1.size());
    const auto& buf = std::get<0>(result);
    const std::vector<value_type> v2(buf.cbegin(), buf.cbegin() + v1.size());
    BOOST_TEST(v2 == v1, boost::test_tools::per_element());
  }

  // 受信の終了
  io_service.stop();
  t.join();
}

BOOST_AUTO_TEST_SUITE_END()
