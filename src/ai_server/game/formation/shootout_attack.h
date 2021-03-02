#ifndef AI_SERVER_GAME_FORMATION_SHOOTOUT_ATTACK_H
#define AI_SERVER_GAME_FORMATION_SHOOTOUT_ATTACK_H

#include <vector>

#include "base.h"

namespace ai_server::game::formation {

class shootout_attack : public v2::base {
public:
  shootout_attack(context& ctx, const std::vector<unsigned int>& ids,
                  const unsigned int keeper_id);

  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  const std::vector<unsigned int> ids_;
  const unsigned int keeper_id_;
  constexpr static unsigned int shootout_id_ = 4; //シュートアウト
};

} // namespace ai_server::game::formation

#endif
