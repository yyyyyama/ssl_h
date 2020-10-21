#ifndef AI_SERVER_GAME_FORMATION_SETPLAY_ATTACK_H
#define AI_SERVER_GAME_FORMATION_SETPLAY_ATTACK_H

#include <chrono>
#include <vector>

#include "ai_server/game/agent/defense.h"
#include "ai_server/game/agent/setplay.h"

#include "base.h"

namespace ai_server::game::formation {

class setplay_attack : public v2::base {
public:
  setplay_attack(context& ctx, const std::vector<unsigned int>& ids, unsigned int keeper_id);

  std::vector<std::shared_ptr<action::base>> execute() override;

  bool finished() const;

private:
  std::size_t decide_wall_count(std::size_t num) const;
  std::pair<std::vector<unsigned int>, std::vector<unsigned int>> divide_wall(
      std::vector<unsigned int> visible_robots, std::size_t wall_count) const;
  std::pair<unsigned int, std::vector<unsigned int>> divide_kicker(
      std::vector<unsigned int>& fw_ids) const;

  std::vector<unsigned int> ids_;
  const unsigned int keeper_id_;
  std::shared_ptr<agent::setplay> setplay_;
  std::shared_ptr<agent::defense> defense_;
  std::chrono::steady_clock::time_point lost_point_;
};

} // namespace ai_server::game::formation

#endif
