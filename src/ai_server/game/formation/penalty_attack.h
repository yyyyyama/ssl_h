#ifndef AI_SERVER_GAME_FORMATION_PENALTY_ATTACK_H
#define AI_SERVER_GAME_FORMATION_PENALTY_ATTACK_H

#include <vector>
#include <Eigen/Core>

#include "base.h"

namespace ai_server::game::agent {
class defense;
class penalty_kick;
} // namespace ai_server::game::agent

namespace ai_server::game::formation {

class penalty_attack : public v2::base {
public:
  penalty_attack(context& ctx, const std::vector<unsigned int>& ids,
                 const unsigned int keeper_id, const unsigned int enemy_keeper,
                 const bool is_start);

  std::vector<std::shared_ptr<action::base>> execute() override;

  bool finished() const;

private:
  const std::vector<unsigned int> ids_;
  const unsigned int keeper_id_;
  const unsigned int enemy_keeper_;
  bool is_start_;
  std::shared_ptr<agent::penalty_kick> pk_;
  std::shared_ptr<agent::defense> df_;
};

} // namespace ai_server::game::formation

#endif
