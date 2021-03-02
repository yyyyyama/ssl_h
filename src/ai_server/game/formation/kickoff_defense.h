#ifndef AI_SERVER_GAME_FORMATION_KICKOFF_DEFENSE_H
#define AI_SERVER_GAME_FORMATION_KICKOFF_DEFENSE_H

#include <array>
#include <vector>
#include <Eigen/Core>

#include "base.h"

namespace ai_server::game::agent {
class defense;
class kick_off_waiter;
} // namespace ai_server::game::agent

namespace ai_server::game::formation {

class kickoff_defense : public v2::base {
public:
  kickoff_defense(context& ctx, const std::vector<unsigned int>& ids,
                  const unsigned int keeper_id);

  std::vector<std::shared_ptr<action::base>> execute() override;

  bool finished() const;

private:
  const std::vector<unsigned int> ids_;
  const unsigned int keeper_id_;
  std::shared_ptr<agent::defense> defense_;
  std::shared_ptr<agent::kick_off_waiter> kickoff_waiter_;
  bool kicked_;
  std::array<Eigen::Vector2d, 5> past_ball_;
  Eigen::Vector2d previous_ball_;
};

} // namespace ai_server::game::formation

#endif
