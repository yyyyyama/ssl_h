#ifndef AI_SERVER_GAME_AGENT_STOPGAME_H
#define AI_SERVER_GAME_AGENT_STOPGAME_H
#include "ai_server/game/agent/base.h"

namespace ai_server {
namespace game {
namespace agent {

class stopgame : public base {
private:
  const std::vector<unsigned int> ids_;
  unsigned int nearest_robot_;

public:
  stopgame(context& ctx, const std::vector<unsigned int>& ids);
  std::vector<std::shared_ptr<action::base>> execute() override;
};

} // namespace agent
} // namespace game
} // namespace ai_server
#endif
