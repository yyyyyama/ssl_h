#ifndef AI_SERVER_SENDER_GRSIM_H
#define AI_SERVER_SENDER_GRSIM_H

#include <string>
#include <boost/asio.hpp>

#include "ai_server/model/command.h"
#include "ai_server/util/net/multicast/sender.h"
#include "ssl-protos/grsim/packet.pb.h"

#include "base.h"

namespace ai_server {
namespace sender {

class grsim final : public base {
public:
  grsim(boost::asio::io_context& io_context, const std::string& grsim_addr, short port);

  void send_command(const model::command& command) override;

private:
  util::net::multicast::sender udp_sender_;
};
} // namespace sender
} // namespace ai_server
#endif
