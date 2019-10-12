#ifndef AI_SERVER_UTIL_NET_MULTICAST_SENDER_H
#define AI_SERVER_UTIL_NET_MULTICAST_SENDER_H

#include <array>
#include <string>
#include <boost/asio.hpp>

namespace ai_server {
namespace util {
namespace net {
namespace multicast {

/// @class   sender
/// @brief   UDP multicastで送信するクラス
class sender {
public:
  /// @brief                  コンストラクタ
  /// @param multicast_addr   マルチキャストアドレス
  /// @param port             ポート
  sender(boost::asio::io_context& io_context, const std::string& multicast_addr, short port);

  /// @brief                  コンストラクタ
  /// @param multicast_addr   マルチキャストアドレス
  /// @param port             ポート
  sender(boost::asio::io_context& io_context, const boost::asio::ip::address& multicast_addr,
         short port);

  /// @brief                  buffer を送信する
  /// @param buffer           送信するデータ
  template <class Buffer>
  void send(Buffer&& buffer) {
    // TODO: 送信できたデータサイズをちゃんと確認する
    socket_.send_to(boost::asio::buffer(std::forward<Buffer>(buffer)), endpoint_);
  }

  /// @brief                  buffer を size だけ送信する
  /// @param buffer           送信するデータ
  /// @param size             送信するデータのサイズ
  template <class Buffer>
  void send(Buffer&& buffer, std::size_t size) {
    socket_.send_to(boost::asio::buffer(std::forward<Buffer>(buffer), size), endpoint_);
  }

private:
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;
};

} // namespace multicast
} // namespace net
} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_NET_MULTICAST_SENDER_H
