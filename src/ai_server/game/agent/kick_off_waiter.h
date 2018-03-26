#ifndef AI_SERVER_GAME_AGENT_KICK_OFF_WAITER_H
#define AI_SERVER_GAME_AGENT_KICK_OFF_WAITER_H

#include "base.h"
#include "ai_server/game/action/move.h"

#include <memory>
#include <vector>

namespace ai_server {
namespace game {
namespace agent {

class kick_off_waiter : public base {
public:
  enum class kickoff_mode { attack, defense };
  kick_off_waiter(const model::world& world, bool is_yellow,
                  const std::vector<unsigned int>& ids);
  std::vector<std::shared_ptr<action::base>> execute() override;
  void set_mode(kickoff_mode mode);
  kickoff_mode mode();

private:
  const std::vector<unsigned int> ids_;
  kickoff_mode mode_;
};
} // namespace agent
} // namespace game
} // namespace ai_server

#endif