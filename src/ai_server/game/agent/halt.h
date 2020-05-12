#ifndef AI_SERVER_GAME_AGENT_HALT_H
#define AI_SERVER_GAME_AGENT_HALT_H

#include "base.h"
#include "ai_server/game/action/no_operation.h"

#include <memory>
#include <vector>

namespace ai_server {
namespace game {
namespace agent {

class halt : public base {
public:
  halt(context& ctx, const std::vector<unsigned int>& ids);
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  std::vector<unsigned int> ids_;
};
} // namespace agent
} // namespace game
} // namespace ai_server

#endif