#ifndef AI_SERVER_GAME_FORMATION_STOPGAME_H
#define AI_SERVER_GAME_FORMATION_STOPGAME_H

#include <memory>
#include <vector>

#include "ai_server/game/agent/defense.h"
#include "ai_server/game/agent/stopgame.h"

#include "base.h"

namespace ai_server::game::formation {

class stopgame : public v2::base {
public:
  stopgame(context& ctx, const std::vector<unsigned int>& ids, unsigned int keeper_id);

  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  std::vector<unsigned int> ids_;
  const unsigned int keeper_id_;
  std::vector<unsigned int> wall_;
};

} // namespace ai_server::game::formation

#endif // AI_SERVER_GAME_FORMATION_STOPGAME_H
