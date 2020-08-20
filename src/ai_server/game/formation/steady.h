#ifndef AI_SERVER_GAME_FORMATION_STEADY_H
#define AI_SERVER_GAME_FORMATION_STEADY_H

#include <memory>
#include <vector>

#include "ai_server/game/agent/all.h"

#include "base.h"

namespace ai_server::game::formation {

class steady : public v2::base {
public:
  steady(context& ctx, const std::vector<unsigned int>& ids, unsigned int keeper_id);

  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  agent::all steady_;
};

} // namespace ai_server::game::formation

#endif // AI_SERVER_GAME_FORMATION_STEADY_H
