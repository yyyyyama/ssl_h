#ifndef AI_SERVER_SENDER_GRSIM_H
#define AI_SERVER_SENDER_GRSIM_H

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <tuple>
#include <boost/variant.hpp>

#include "ai_server/model/command.h"
#include "ssl-protos/grsim/packet.pb.h"
#include "ai_server/util/net/multicast/sender.h"
#include "base.h"

namespace ai_server {
namespace sender {

class grsim final : public base {
public:
  grsim(boost::asio::io_service& io_service, const std::string& grsim_addr, short port);

  void send_command(const model::command& command) override;

private:
  util::net::multicast::sender udp_sender_;
};
}
}
#endif
