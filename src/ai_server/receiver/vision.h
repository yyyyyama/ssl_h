#ifndef AI_SERVER_RECEIVER_VISION_H
#define AI_SERVER_RECEIVER_VISION_H

#include <string>
#include <boost/asio.hpp>
#include <boost/signals2.hpp>

#include "ai_server/util/net/multicast/receiver.h"

#include "ssl-protos/vision/wrapperpacket.pb.h"

namespace ai_server {
namespace receiver {

/// @class   vision
/// @brief   SSL-Visionからデータを受信するクラス
class vision {
  // データ受信時に呼ぶシグナルの型
  using receive_signal_t = boost::signals2::signal<void(const ssl_protos::vision::Packet&)>;
  // エラー時に呼ぶシグナルの型
  using error_signal_t = boost::signals2::signal<void(void)>;

public:
  /// @brief                  コンストラクタ
  /// @param listen_addr      通信に使うインターフェースのIPアドレス
  /// @param multicast_addr   マルチキャストアドレス
  /// @param port             ポート
  vision(boost::asio::io_context& io_context, const std::string& listen_addr,
         const std::string& multicast_addr, short port);

  /// @brief                  データ受信時に slot が呼ばれるようにする
  /// @param slot             データ受信時に呼びたい関数オブジェクト
  boost::signals2::connection on_receive(const receive_signal_t::slot_type& slot);

  /// @brief                  エラー時に slot が呼ばれるようにする
  /// @param slot             エラー時に呼びたい関数オブジェクト
  boost::signals2::connection on_error(const error_signal_t::slot_type& slot);

private:
  void parse_packet(const util::net::multicast::receiver::buffer_t& buffer, std::size_t size);

  receive_signal_t received_;
  error_signal_t errored_;

  util::net::multicast::receiver receiver_;
};

} // namespace receiver
} // namespace ai_server

#endif // AI_SERVER_RECEIVER_VISION_H
