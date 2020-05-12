#ifndef AI_SERVER_GAME_AGENT_ALIGNMENT_H
#define AI_SERVER_GAME_AGENT_ALIGNMENT_H

#include "base.h"
#include "ai_server/game/action/move.h"

namespace ai_server {
namespace game {
namespace agent {

class alignment : public base {
public:
  alignment(context& ctx, const std::vector<unsigned int>& ids);
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  const std::vector<unsigned int> ids_;
};
} // namespace agent
} // namespace game
} // namespace ai_server

#endif
