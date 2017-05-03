#ifndef AI_SERVER_GAME_AGENT_REGULAR_H
#define AI_SERVER_GAME_AGENT_REGULAR_H

#include "base.h"
#include "ai_server/game/action/marking.h"
#include "ai_server/game/action/move.h"
//#include "ai_server/game/action/chase_ball.h"

namespace ai_server {
namespace game {
namespace agent {

class regular : public base {
public:
  regular(const model::world& world, bool is_yellow, const std::vector<unsigned int>& ids);

  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  const std::vector<unsigned int>& ids_;

  // Action
  std::shared_ptr<action::marking> marking_;
  // std::shared_ptr<action::chase_ball> chase_ball_;
  
};

} // agent
} // game
} // ai_server

#endif
