#ifndef AI_SERVER_GAME_ACTION_DEFENSE_H
#define AI_SERVER_GAME_ACTION_DEFENSE_H

#include <vector>
#include <memory>

#include "ai_server/game/agent/base.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/move_velocity.h"

namespace ai_server {
namespace game {
namespace agent {
class defense : public base {
public:
  defense(const model::world& world, bool is_yellow, unsigned int keeper_id,
          const std::vector<unsigned int>& wall_ids);
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  unsigned int keeper_id_;
  const std::vector<unsigned int>& wall_ids_;
  std::vector<std::shared_ptr<action::move>> wall_;
  std::shared_ptr<action::move> keeper_;
  std::vector<double> target_x;
  std::vector<double> target_y;
  double x_;
  double y_;
};
}
}
}
#endif // AI_SERVER_GAME_ACTION_DEFENSE_H
