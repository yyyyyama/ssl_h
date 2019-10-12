#include <functional>
#include "vision.h"

namespace ai_server {
namespace receiver {

vision::vision(boost::asio::io_context& io_context, const std::string& listen_addr,
               const std::string& multicast_addr, short port)
    : receiver_(io_context, listen_addr, multicast_addr, port) {
  // Google Protocol Buffersライブラリのバージョンをチェックする
  // 互換性のないバージョンが使われていた場合は例外吐いて落ちる()
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  // multicastクライアントとparse_packet()をつなげる
  receiver_.on_receive(
      std::bind(&vision::parse_packet, this, std::placeholders::_1, std::placeholders::_2));
}

boost::signals2::connection vision::on_receive(const receive_signal_t::slot_type& slot) {
  return received_.connect(slot);
}

boost::signals2::connection vision::on_error(const error_signal_t::slot_type& slot) {
  return errored_.connect(slot);
}

void vision::parse_packet(const util::net::multicast::receiver::buffer_t& buffer,
                          std::size_t size) {
  ssl_protos::vision::Packet packet;

  // パケットをパース
  if (packet.ParseFromArray(buffer.data(), size)) {
    // 成功したら登録された関数を呼び出す
    received_(packet);
  } else {
    errored_();
  }
}

} // namespace receiver
} // namespace ai_server
