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
  enum class keeper_mode { normal_mode, pk_mode };
  void set_mode(action::defense::keeper_mode mode);
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  unsigned int keeper_id_;
  const std::vector<unsigned int>& wall_ids_;
  std::vector<std::shared_ptr<action::move>> wall_;
  std::shared_ptr<action::move> keeper_;
  double x_         = 0.0;
  double y_         = 0.0;
  bool flag_        = true;
  keeper_mode mode_ = keeper_mode::normal_mode;
};
}
}
}
#endif // AI_SERVER_GAME_ACTION_DEFENSE_H
