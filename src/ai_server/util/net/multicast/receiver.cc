#include <chrono>
#include <functional>
#include "receiver.h"

using namespace std::chrono_literals;

namespace ai_server {
namespace util {
namespace net {
namespace multicast {

receiver::receiver(boost::asio::io_context& io_context, const std::string& multicast_addr,
                   unsigned short port)
    : receiver(io_context, boost::asio::ip::make_address(multicast_addr), port) {}

receiver::receiver(boost::asio::io_context& io_context,
                   const boost::asio::ip::address& multicast_addr, unsigned short port)
    : total_messages_{0}, socket_{io_context}, timer_{io_context} {
  boost::asio::ip::udp::endpoint listen_endpoint{
      multicast_addr.is_v4() ? boost::asio::ip::udp::v4() : boost::asio::ip::udp::v6(), port};

  // ソケットを開く
  socket_.open(listen_endpoint.protocol());

  // 既に使われているポートにも bind できるようにする
  socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));

  socket_.bind(listen_endpoint);

  // multicast グループに join
  socket_.set_option(boost::asio::ip::multicast::join_group(multicast_addr));

  // receive_data, count_messages_per_second を coroutine で動かす
  boost::asio::spawn(socket_.get_executor(), [&](auto yield) { receive_data(yield); });
  boost::asio::spawn(timer_.get_executor(),
                     [&](auto yield) { count_messages_per_second(yield); });
}

receiver::receiver(boost::asio::io_context& io_context,
                   const boost::asio::ip::address_v4& multicast_addr, unsigned short port,
                   const boost::asio::ip::address_v4& interface_addr)
    : total_messages_{0}, socket_{io_context}, timer_{io_context} {
  boost::asio::ip::udp::endpoint listen_endpoint{boost::asio::ip::udp::v4(), port};

  socket_.open(listen_endpoint.protocol());
  socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
  socket_.bind(listen_endpoint);
  socket_.set_option(boost::asio::ip::multicast::join_group(multicast_addr, interface_addr));

  boost::asio::spawn(socket_.get_executor(), [&](auto yield) { receive_data(yield); });
  boost::asio::spawn(timer_.get_executor(),
                     [&](auto yield) { count_messages_per_second(yield); });
}

void receiver::receive_data(boost::asio::yield_context yield) {
  boost::system::error_code ec;

  for (;;) {
    buffer_t data;
    const auto recieved =
        socket_.async_receive_from(boost::asio::buffer(data), endpoint_, yield[ec]);

    if (ec) {
      call_error_callback(ec);
    } else {
      auto time = std::chrono::system_clock::now();
      call_receive_callback(data, recieved, ++total_messages_, time);
    }
  }
}

void receiver::count_messages_per_second(boost::asio::yield_context yield) {
  boost::system::error_code ec;

  auto prev_total_messages = total_messages_;

  timer_.expires_after(1s);

  for (;;) {
    // 設定された時刻まで待つ
    timer_.async_wait(yield[ec]);
    if (ec) {
      call_error_callback(ec);
      break;
    }

    // 1秒間で受信したメッセージ数を求めて更新する
    const auto tm = total_messages_;
    call_status_updated_callback(tm - prev_total_messages);
    prev_total_messages = tm;

    // 次に更新を行う時刻 (1秒後) を設定
    timer_.expires_after(1s);
  }
}

} // namespace multicast
} // namespace net
} // namespace util
} // namespace ai_server
