#include "sender.h"

namespace ai_server {
namespace util {
namespace net {
namespace multicast {

sender::sender(boost::asio::io_context& io_context, const std::string& multicast_addr,
               short port)
    : sender(io_context, boost::asio::ip::address::from_string(multicast_addr), port) {}

sender::sender(boost::asio::io_context& io_context,
               const boost::asio::ip::address& multicast_addr, short port)
    : socket_(io_context), endpoint_(multicast_addr, port) {
  // ソケットの初期化
  socket_.open(endpoint_.protocol());
}

} // namespace multicast
} // namespace net
} // namespace util
} // namespace ai_server
