#define BOOST_TEST_DYN_LINK

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <stdexcept>

#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

extern "C" {
#include <pty.h>
#include <unistd.h>
}

#include "ai_server/radio/connection/serial.h"

#include "test_helpers/asio.h"

namespace radio = ai_server::radio;

using namespace std::string_literals;
using namespace std::chrono_literals;

class pty {
  int fd_;

public:
  pty(int fd) : fd_{fd} {}
  pty(pty&& p) noexcept : fd_{std::exchange(p.fd_, -1)} {}
  pty()           = delete;
  pty(const pty&) = delete;

  ~pty() {
    if (fd_ != -1) ::close(fd_);
  }

  int fd() const {
    return fd_;
  }
  std::string name() const {
    return ::ttyname(fd_);
  }

  static std::pair<pty, pty> openpty() {
    int master, slave;
    if (::openpty(&master, &slave, nullptr, nullptr, nullptr) < 0) {
      throw std::runtime_error{"openpty failed"};
    }
    return {std::piecewise_construct, std::forward_as_tuple(master),
            std::forward_as_tuple(slave)};
  }
};

BOOST_AUTO_TEST_SUITE(serial)

BOOST_AUTO_TEST_CASE(send, *boost::unit_test::timeout(30)) {
  auto [master, slave] = pty::openpty();

  boost::asio::io_context ctx1{};
  radio::connection::serial tx{ctx1, slave.name()};
  auto th = run_io_context_in_new_thread(ctx1);

  BOOST_TEST(tx.total_messages() == 0);
  BOOST_TEST(tx.messages_per_second() == 0);
  BOOST_TEST(tx.total_errors() == 0);
  BOOST_TEST((tx.last_sent() == std::chrono::system_clock::time_point{}));

  boost::asio::io_context ctx2{};
  boost::asio::posix::stream_descriptor rx{ctx2, master.fd()};

  std::array<char, 4096> buf{};

  {
    tx.send("Hello"s);

    auto len = rx.read_some(boost::asio::buffer(buf));

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

    auto len = rx.read_some(boost::asio::buffer(buf));

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
