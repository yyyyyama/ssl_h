#include <functional>
#include "refbox.h"

namespace ai_server {
namespace receiver {

refbox::refbox(boost::asio::io_service& io_service, const std::string& listen_addr,
               const std::string& multicast_addr, short port)
    : receiver_(io_service, listen_addr, multicast_addr, port) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  receiver_.on_receive(
      std::bind(&refbox::parse_packet, this, std::placeholders::_1, std::placeholders::_2));
}

boost::signals2::connection refbox::on_receive(const receive_signal_t::slot_type& slot) {
  return received_.connect(slot);
}

boost::signals2::connection refbox::on_error(const error_signal_t::slot_type& slot) {
  return errored_.connect(slot);
}

void refbox::parse_packet(const util::net::multicast::receiver::buffer_t& buffer,
                          std::size_t size) {
  ssl_protos::refbox::Referee packet;

  if (packet.ParseFromArray(buffer.data(), size)) {
    received_(packet);
  } else {
    errored_();
  }
}
}
}
