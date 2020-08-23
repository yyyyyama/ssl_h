#ifndef AI_SERVER_GAME_FORMATION_TIMEOUT_H
#define AI_SERVER_GAME_FORMATION_TIMEOUT_H

#include <memory>
#include <vector>

#include "ai_server/game/agent/alignment.h"

#include "base.h"

namespace ai_server::game::formation {

class timeout : public v2::base {
public:
  timeout(context& ctx, const std::vector<unsigned int>& ids);

  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  agent::alignment timeout_;
};

} // namespace ai_server::game::formation

#endif // AI_SERVER_GAME_FORMATION_TIMEOUT_H
