#ifndef AI_SERVER_GAME_ACTION_DEFENSE_H
#define AI_SERVER_GAME_ACTION_DEFENSE_H

#include <vector>
#include <memory>

#include "ai_server/game/agent/base.h"
#include "ai_server/game/action/move.h"

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
  std::vector<std::shared_ptr<action::move>> wall_;
  std::shared_ptr<action::move> keeper_;
  double x_;
  double y_;
  defense_mode mode_;
};
}
}
}
#endif // AI_SERVER_GAME_ACTION_DEFENSE_H
