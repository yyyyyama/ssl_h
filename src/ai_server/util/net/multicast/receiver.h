#ifndef AI_SERVER_UTIL_NET_MULTICAST_RECEIVER_H
#define AI_SERVER_UTIL_NET_MULTICAST_RECEIVER_H

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#define BOOST_COROUTINES_NO_DEPRECATION_WARNING
#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>

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

  // データ受信時のコールバック関数の型
  using receive_callback_type =
      std::function<void(const buffer_t&, // 受信バッファ
                         std::size_t,     // データサイズ
                         std::uint64_t,   // 受信したメッセージの総数
                         std::chrono::system_clock::time_point // メッセージを受信した時刻
                         )>;
  // 受信状況更新時のコールバック関数の型
  using status_callback_type = std::function<void(std::uint64_t // 1秒間に受信したメッセージの数
                                                  )>;
  // エラー時のコールバック関数の型
  using error_callback_type = std::function<void(const boost::system::error_code&)>;

  /// @brief                  コンストラクタ
  /// @param listen_addr      通信に使うインターフェースのIPアドレス
  /// @param multicast_addr   マルチキャストアドレス
  /// @param port             ポート
  ///
  /// \p listen_addr, \p multicast_addr は \p boost::asio::ip::make_address によって
  /// \p boost::asio::ip::address に変換される。
  /// 失敗した場合 \p boost::system::system_error 例外が発生する
  receiver(boost::asio::io_context& io_context, const std::string& listen_addr,
           const std::string& multicast_addr, unsigned short port);

  /// @brief                  コンストラクタ
  /// @param listen_addr      通信に使うインターフェースのIPアドレス
  /// @param multicast_addr   マルチキャストアドレス
  /// @param port             ポート
  receiver(boost::asio::io_context& io_context, const boost::asio::ip::address& listen_addr,
           const boost::asio::ip::address& multicast_addr, unsigned short port);

  /// @brief データ受信時に呼ばれるコールバック関数を登録する
  inline void on_receive(receive_callback_type cb) {
    receive_callback_ = std::move(cb);
  }

  /// @brief 受信状況更新時に呼ばれるコールバック関数を登録する
  inline void on_status_updated(status_callback_type cb) {
    status_updated_callback_ = std::move(cb);
  }

  /// @brief エラー時に呼ばれるコールバック関数を登録する
  inline void on_error(error_callback_type cb) {
    error_callback_ = std::move(cb);
  }

private:
  /// @brief \p addr に接続してデータを受信する
  void receive_data(boost::asio::yield_context yield,
                    const boost::asio::ip::udp::endpoint& endpoint,
                    const boost::asio::ip::address& addr);

  /// @brief 1秒間に受信したメッセージを数える
  void count_messages_per_second(boost::asio::yield_context yield);

  inline void call_receive_callback(const buffer_t& buffer, std::size_t length,
                                    std::uint64_t total_messages,
                                    std::chrono::system_clock::time_point time) {
    if (receive_callback_) receive_callback_(buffer, length, total_messages, time);
  }

  inline void call_status_updated_callback(std::uint64_t messages_per_second) {
    if (status_updated_callback_) status_updated_callback_(messages_per_second);
  }

  inline void call_error_callback(const boost::system::error_code& ec) {
    if (error_callback_) error_callback_(ec);
  }

  /// 受信したメッセージの総数
  std::uint64_t total_messages_;

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;

  boost::asio::steady_timer timer_;

  receive_callback_type receive_callback_;
  status_callback_type status_updated_callback_;
  error_callback_type error_callback_;
};

} // namespace multicast
} // namespace net
} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_NET_MULTICAST_RECEIVER_H
