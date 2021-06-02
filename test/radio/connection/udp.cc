#define BOOST_TEST_DYN_LINK

#include <array>
#include <chrono>
#include <future>
#include <string>
#include <thread>
#include <tuple>

#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/radio/connection/udp.h"

#include "test_helpers/asio.h"

namespace radio = ai_server::radio;

using namespace std::string_literals;
using namespace std::chrono_literals;

static constexpr unsigned short port = 31337;

BOOST_AUTO_TEST_SUITE(udp)

BOOST_AUTO_TEST_CASE(send, *boost::unit_test::timeout(30)) {
  boost::asio::io_context ctx1{};

  radio::connection::udp_tx tx{ctx1, {boost::asio::ip::udp::v4(), port}};
  auto th = run_io_context_in_new_thread(ctx1);

  BOOST_TEST(tx.total_messages() == 0);
  BOOST_TEST(tx.messages_per_second() == 0);
  BOOST_TEST(tx.total_errors() == 0);
  BOOST_TEST((tx.last_sent() == std::chrono::system_clock::time_point{}));

  boost::asio::io_context ctx2{};
  boost::asio::ip::udp::socket rx{ctx2, {boost::asio::ip::udp::v4(), port}};

  std::array<char, 4096> buf{};

  {
    tx.send("Hello"s);

    boost::asio::ip::udp::endpoint e{};
    auto len = rx.receive_from(boost::asio::buffer(buf), e);

    // send したものと同じか
    BOOST_TEST((std::string{buf.cbegin(), buf.cbegin() + len}) == "Hello"s);

    // 値が更新されているか
    BOOST_TEST(tx.total_messages() == 1);
    BOOST_TEST(tx.total_errors() == 0);
    BOOST_TEST((std::chrono::system_clock::now() - tx.last_sent() < 1s));
  }

  {
    std::vector<char> b{'A', 'B', 'C', '\0', 'a', 'b', 'c'};

    tx.send(b);

    boost::asio::ip::udp::endpoint e{};
    auto len = rx.receive_from(boost::asio::buffer(buf), e);

    // send したものと同じか
    BOOST_TEST((std::vector<char>{buf.cbegin(), buf.cbegin() + len}) == b,
               boost::test_tools::per_element());

    // 値が更新されているか
    BOOST_TEST(tx.total_messages() == 2);
    BOOST_TEST(tx.total_errors() == 0);
    BOOST_TEST((std::chrono::system_clock::now() - tx.last_sent() < 1s));
  }

  // 前回の1秒間に受信したメッセージは2
  BOOST_TEST(tx.messages_per_second() == 0);
  std::this_thread::sleep_for(1s);
  BOOST_TEST(tx.messages_per_second() == 2);
}

using promise_type = std::promise<std::tuple<boost::system::error_code, std::size_t>>;

BOOST_AUTO_TEST_CASE(send_and_recv, *boost::unit_test::timeout(30)) {
  boost::asio::io_context ctx1{};

  radio::connection::udp con{ctx1, {boost::asio::ip::udp::v4(), port}};
  auto th = run_io_context_in_new_thread(ctx1);

  boost::asio::io_context ctx2{};
  boost::asio::ip::udp::socket sock{ctx2, {boost::asio::ip::udp::v4(), port}};

  std::array<char, 4096> buf{};

  {
    // con -> sock
    const auto msg1 = "Hello"s;
    con.send(msg1);

    boost::asio::ip::udp::endpoint sender{};
    const auto l1 = sock.receive_from(boost::asio::buffer(buf), sender);
    BOOST_TEST(l1 == msg1.size());
    BOOST_TEST((std::string{buf.cbegin(), buf.cbegin() + l1}) == msg1);

    // sock -> con
    promise_type p{};
    con.recv(buf,
             [&p](auto&&... args) { p.set_value({std::forward<decltype(args)>(args)...}); });

    const auto msg2 = "Bonjour"s;
    sock.send_to(boost::asio::buffer(msg2), sender);

    const auto [ec, l2] = p.get_future().get();
    BOOST_TEST(!ec.failed());
    BOOST_TEST(l2 == msg2.size());
    BOOST_TEST((std::string{buf.cbegin(), buf.cbegin() + l2}) == msg2);
  }
}

BOOST_AUTO_TEST_CASE(recv_multicast, *boost::unit_test::timeout(30)) {
  boost::asio::io_context ctx1{};

  boost::asio::ip::udp::endpoint ep{boost::asio::ip::make_address("224.5.0.1"), port};
  radio::connection::udp con{ctx1, ep};
  auto th = run_io_context_in_new_thread(ctx1);

  boost::asio::io_context ctx2{};
  boost::asio::ip::udp::socket sock{ctx2, ep.protocol()};

  std::array<char, 4096> buf{};

  {
    promise_type p{};
    con.recv(buf,
             [&p](auto&&... args) { p.set_value({std::forward<decltype(args)>(args)...}); });

    sock.send_to(boost::asio::buffer("Hello"s), ep);

    const auto [ec, len] = p.get_future().get();
    BOOST_TEST(!ec.failed());
    BOOST_TEST((std::string{buf.cbegin(), buf.cbegin() + len}) == "Hello"s);
  }

  {
    promise_type p{};
    con.recv(buf,
             [&p](auto&&... args) { p.set_value({std::forward<decltype(args)>(args)...}); });

    con.send("Hello"s);

    // 自分が送ったメッセージは受信側に流れてこない
    auto f = p.get_future();
    BOOST_TEST((f.wait_for(100ms) == std::future_status::timeout));
  }
}

BOOST_AUTO_TEST_CASE(send2, *boost::unit_test::timeout(30)) {
  boost::asio::io_context ctx1{};

  // ホスト名を解決できるタイプのコンストラクタ
  radio::connection::udp tx{ctx1, "localhost", port};
  auto th = run_io_context_in_new_thread(ctx1);

  boost::asio::io_context ctx2{};
  boost::asio::ip::udp::socket rx{ctx2, {tx.endpoint().protocol(), port}};

  std::array<char, 4096> buf{};

  {
    tx.send("Hello"s);

    boost::asio::ip::udp::endpoint e{};
    auto len = rx.receive_from(boost::asio::buffer(buf), e);

    BOOST_TEST((std::string{buf.cbegin(), buf.cbegin() + len}) == "Hello"s);
  }
}

BOOST_AUTO_TEST_SUITE_END()
