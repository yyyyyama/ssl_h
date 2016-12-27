#include <functional>
#include "receiver.h"

namespace ai_server {
namespace util {
namespace net {
namespace multicast {

receiver::receiver(boost::asio::io_service& io_service, const std::string& listen_addr,
                   const std::string& multicast_addr, short port)
    : receiver(io_service, boost::asio::ip::address::from_string(listen_addr),
               boost::asio::ip::address::from_string(multicast_addr), port) {}

receiver::receiver(boost::asio::io_service& io_service,
                   const boost::asio::ip::address& listen_addr,
                   const boost::asio::ip::address& multicast_addr, short port)
    : socket_(io_service) {
  boost::asio::ip::udp::endpoint listen_endpoint(listen_addr, port);

  // ソケットの初期化
  socket_.open(listen_endpoint.protocol());
  socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
  socket_.bind(listen_endpoint);

  // multicastグループにjoinする
  socket_.set_option(boost::asio::ip::multicast::join_group(multicast_addr));

  // コルーチンを作成し, データを受信する
  boost::asio::spawn(socket_.get_io_service(),
                     std::bind(&receiver::receive_data, this, std::placeholders::_1));
}

boost::signals2::connection receiver::on_receive(const receive_signal_t::slot_type& slot) {
  return on_receive_.connect(slot);
}

boost::signals2::connection receiver::on_error(const error_signal_t::slot_type& slot) {
  return on_error_.connect(slot);
}

void receiver::receive_data(boost::asio::yield_context yield) {
  boost::system::error_code ec;

  for (;;) {
    buffer_t data;
    const auto recieved =
        socket_.async_receive_from(boost::asio::buffer(data), endpoint_, yield[ec]);

    if (ec) {
      on_error_(ec);
    } else {
      on_receive_(data, recieved);
    }
  }
}

} // namespace multicast
} // namespace net
} // namespace util
} // namespace ai_server
