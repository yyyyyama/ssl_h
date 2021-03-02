#ifndef AI_SERVER_GAME_FORMATION_KICKOFF_ATTACK_H
#define AI_SERVER_GAME_FORMATION_KICKOFF_ATTACK_H

#include <vector>

#include "base.h"

namespace ai_server::game::agent {
class defense;
class kick_off;
} // namespace ai_server::game::agent

namespace ai_server::game::formation {

class kickoff_attack : public v2::base {
public:
  kickoff_attack(context& ctx, const std::vector<unsigned int>& ids,
                 const unsigned int keeper_id, const bool is_start);

  std::vector<std::shared_ptr<action::base>> execute() override;

  bool finished() const;

private:
  const std::vector<unsigned int> ids_;
  const unsigned int keeper_id_;
  const bool is_start_;
  std::shared_ptr<agent::defense> defense_;
  std::shared_ptr<agent::kick_off> kickoff_;
};

} // namespace ai_server::game::formation

#endif
