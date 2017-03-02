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
  using flow_control_t = boost::asio::serial_port::flow_control::type;
  using parity_t       = boost::asio::serial_port::parity::type;
  using stop_bits_t    = boost::asio::serial_port::stop_bits::type;

  /// @brief                  コンストラクタ
  /// @param device           利用するシリアルポートのデバイス名
  serial(boost::asio::io_service& io_service, const std::string& device);

  /// @brief                  ボーレートを取得する
  unsigned int baud_rate();

  /// @brief                  データビット(1バイトのビット数)を取得する
  unsigned int character_size();

  /// @brief                  フロー制御のタイプを取得する
  flow_control_t flow_control();

  /// @brief                  パリティビットを取得する
  parity_t parity();

  /// @brief                  ストップビットを取得する
  stop_bits_t stop_bits();

  /// @brief                  ボーレートを設定する
  void set_baud_rate(unsigned int value);

  /// @brief                  データビット(1バイトのビット数)を設定する
  void set_character_size(unsigned int value);

  /// @brief                  フロー制御のタイプを設定する
  void set_flow_control(flow_control_t value);

  /// @brief                  パリティビットを設定する
  void set_parity(parity_t value);

  /// @brief                  ストップビットを設定する
  void set_stop_bits(stop_bits_t value);

  /// @brief                  buffer を送信する
  /// @param buffer           送信するデータ
  template <class Buffer>
  void send(Buffer&& buffer) {
    // TODO: 送信できたデータサイズをちゃんと確認する
    serial_.write_some(boost::asio::buffer(std::forward<Buffer>(buffer)));
  }

  /// @brief                  buffer を size だけ送信する
  /// @param buffer           送信するデータ
  /// @param size             送信するデータのサイズ
  template <class Buffer>
  void send(Buffer&& buffer, std::size_t size) {
    send(boost::asio::buffer(std::forward<Buffer>(buffer), size));
  }

private:
  boost::asio::serial_port serial_;
};

} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_SERIAL_H
