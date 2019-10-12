#ifndef AI_SERVER_UTIL_NET_MULTICAST_RECEIVER_H
#define AI_SERVER_UTIL_NET_MULTICAST_RECEIVER_H

#include <array>
#include <string>

#define BOOST_COROUTINES_NO_DEPRECATION_WARNING
#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/signals2.hpp>

namespace ai_server {
namespace util {
namespace net {
namespace multicast {

/// @class   receiver
/// @brief   UDP multicastを受信するクラス
class receiver {
public:
  /// 受信したデータを格納するバッファのサイズ
  constexpr static std::size_t buffer_size = 4096;

  /// バッファの型
  using buffer_t = std::array<char, buffer_size>;
  // データ受信時に呼ぶシグナルの型
  using receive_signal_t = boost::signals2::signal<void(const buffer_t&, std::size_t)>;
  // エラー時に呼ぶシグナルの型
  using error_signal_t = boost::signals2::signal<void(const boost::system::error_code&)>;

  /// @brief                  コンストラクタ
  /// @param listen_addr      通信に使うインターフェースのIPアドレス
  /// @param multicast_addr   マルチキャストアドレス
  /// @param port             ポート
  receiver(boost::asio::io_context& io_context, const std::string& listen_addr,
           const std::string& multicast_addr, short port);

  /// @brief                  コンストラクタ
  /// @param listen_addr      通信に使うインターフェースのIPアドレス
  /// @param multicast_addr   マルチキャストアドレス
  /// @param port             ポート
  receiver(boost::asio::io_context& io_context, const boost::asio::ip::address& listen_addr,
           const boost::asio::ip::address& multicast_addr, short port);

  /// @brief                  データ受信時に slot が呼ばれるようにする
  /// @param slot             データ受信時に呼びたい関数オブジェクト
  boost::signals2::connection on_receive(const receive_signal_t::slot_type& slot);

  /// @brief                  エラー時に slot が呼ばれるようにする
  /// @param slot             エラー時に呼びたい関数オブジェクト
  boost::signals2::connection on_error(const error_signal_t::slot_type& slot);

private:
  /// @brief                  データを受信する
  void receive_data(boost::asio::yield_context yield);

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;

  receive_signal_t received_;
  error_signal_t errored_;
};

} // namespace multicast
} // namespace net
} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_NET_MULTICAST_RECEIVER_H
