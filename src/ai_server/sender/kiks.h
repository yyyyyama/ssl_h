#ifndef AI_SERVER_SENDER_KIKS_H
#define AI_SERVER_SENDER_KIKS_H

#include <boost/asio.hpp>

#include "ai_server/model/command.h"
#include "ai_server/util/serial.h"
#include "ai_server/sender/base.h"

namespace ai_server {
namespace sender {
class kiks final : public base {
public:
  using data_t = std::array<std::uint8_t, 11>;
  kiks(boost::asio::io_service& io_service, const std::string& device);
  void send_command(const model::command& command) override;

#if defined(AI_SERVER_UNIT_TESTING)
public:
#else
private:
#endif
  kiks::data_t to_data_t(const model::command& command);

private:
  util::serial serial_;
};
}
}

#endif // AI_SERVER_SENDER_KIKS_H
