#ifndef AI_SERVER_SENDER_BASE_H
#define AI_SERVER_SENDER_BASE_H

#include "ai_server/model/command.h"

namespace ai_server {
namespace sender {

class base {
public:
  virtual ~base() = default;

  virtual void send_command(const model::command& command, model::team_color color) = 0;
};

} // namespace sender
} // namespace ai_server

#endif // AI_SERVER_SENDER_BASE_H
