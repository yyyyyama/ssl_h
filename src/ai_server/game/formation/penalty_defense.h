#ifndef AI_SERVER_GAME_FORMATION_PENALTY_DEFENSE_H
#define AI_SERVER_GAME_FORMATION_PENALTY_DEFENSE_H

#include <array>
#include <chrono>
#include <vector>
#include <Eigen/Core>

#include "ai_server/game/agent/penalty_kick.h"

#include "base.h"

namespace ai_server::game::formation {

class penalty_defense : public v2::base {
public:
  penalty_defense(context& ctx, const std::vector<unsigned int>& ids,
                  const unsigned int keeper_id, const unsigned int enemy_keeper);

  std::vector<std::shared_ptr<action::base>> execute() override;

  bool finished() const;

private:
  const std::vector<unsigned int> ids_;
  const unsigned int keeper_id_;
  const unsigned int enemy_keeper_;
  const Eigen::Vector2d previous_ball_;
  bool kicked_;
  bool finished_;
  std::array<Eigen::Vector2d, 5> past_ball_;
  std::chrono::steady_clock::time_point kicked_time_;
};

} // namespace ai_server::game::formation

#endif
