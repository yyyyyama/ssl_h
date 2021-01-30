#ifndef AI_SERVER_GAME_FORMATION_SHOOTOUT_DEFENSE_H
#define AI_SERVER_GAME_FORMATION_SHOOTOUT_DEFENSE_H

#include <vector>

#include "base.h"

namespace ai_server::game::formation {

class shootout_defense : public v2::base {
public:
  shootout_defense(context& ctx, const std::vector<unsigned int>& ids,
                   const unsigned int keeper_id, const unsigned int enemy_keeper);
  std::vector<std::shared_ptr<action::base>> execute() override;

  void start();

private:
  const std::vector<unsigned int> ids_;
  const unsigned int keeper_id_;
  const unsigned int enemy_keeper_;
  bool is_start_;
};

} // namespace ai_server::game::formation

#endif
