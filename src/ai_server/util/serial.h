#ifndef AI_SERVER_UTIL_SERIAL_H
#define AI_SERVER_UTIL_SERIAL_H

#include <array>
#include <string>
#include <boost/asio.hpp>

namespace ai_server {
namespace util {

/// @class   serial
/// @brief   シリアルポートを使った通信を行うクラス
class serial {
public:
  /// @brief                  コンストラクタ
  /// @param device           利用するシリアルポートのデバイス名
  serial(boost::asio::io_service& io_service, const std::string& device);

  /// @brief                  buffer を送信する
  /// @param buffer           送信するデータ
  template <class Buffer>
  void send(const Buffer& buffer) {
    // TODO: 送信できたデータサイズをちゃんと確認する
    serial_.write_some(buffer);
  }

  /// @brief                  buffer を size だけ送信する
  /// @param buffer           送信するデータ
  /// @param size             送信するデータのサイズ
  template <class Buffer>
  void send(const Buffer& buffer, std::size_t size) {
    send(boost::asio::buffer(buffer, size));
  }

private:
  boost::asio::serial_port serial_;
};

} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_SERIAL_H
