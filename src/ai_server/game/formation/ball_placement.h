#ifndef AI_SERVER_GAME_FORMATION_BALL_PLACEMENT_H
#define AI_SERVER_GAME_FORMATION_BALL_PLACEMENT_H

#include <memory>
#include <vector>

#include "ai_server/game/agent/ball_placement.h"

#include "base.h"

namespace ai_server::game::formation {

class ball_placement : public v2::base {
public:
  ball_placement(context& ctx, const std::vector<unsigned int>& ids,
                 const Eigen::Vector2d& target, bool is_active);

  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  agent::ball_placement ball_placement_;
};

} // namespace ai_server::game::formation

#endif // AI_SERVER_GAME_FORMATION_BALL_PLACEMENT_H
