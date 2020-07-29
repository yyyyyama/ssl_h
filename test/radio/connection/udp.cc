#define BOOST_TEST_DYN_LINK

#include <array>
#include <chrono>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/radio/connection/udp.h"

#include "test_helpers/asio.h"

namespace radio = ai_server::radio;

using namespace std::string_literals;
using namespace std::chrono_literals;

BOOST_AUTO_TEST_SUITE(udp)

BOOST_AUTO_TEST_CASE(send, *boost::unit_test::timeout(30)) {
  boost::asio::io_context ctx1{};

  constexpr unsigned short port = 31337;

  radio::connection::udp tx{ctx1, {boost::asio::ip::udp::v4(), port}};
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

BOOST_AUTO_TEST_SUITE_END()
