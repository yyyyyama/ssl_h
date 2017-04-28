#ifndef AI_SERVER_GAME_ACTION_DEFENSE_H
#define AI_SERVER_GAME_ACTION_DEFENSE_H

#include <vector>
#include <memory>
#include <Eigen/Dense>

#include "ai_server/game/agent/base.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/kick_action.h"
#include "ai_server/game/action/vec.h"

namespace ai_server {
namespace game {
namespace agent {
class defense : public base {
public:
  defense(const model::world& world, bool is_yellow, unsigned int keeper_id,
          const std::vector<unsigned int>& wall_ids);
  enum class defense_mode { normal_mode, pk_mode };
  void set_mode(agent::defense::defense_mode mode);
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  unsigned int keeper_id_;
  const std::vector<unsigned int>& wall_ids_;
  std::vector<std::shared_ptr<action::vec>> wall_;
  std::shared_ptr<action::vec> keeper_v_;
  std::shared_ptr<action::kick_action> keeper_k_;
  std::vector<Eigen::Vector2d> target_;
  Eigen::Vector2d orientation_;
  defense_mode mode_;
};
}
}
}
#endif // AI_SERVER_GAME_ACTION_DEFENSE_H
