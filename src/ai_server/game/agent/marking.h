#ifndef AI_SERVER_GAME_AGENT_MARKING_H
#define AI_SERVER_GAME_AGENT_MARKING_H
#include <memory>
#include <vector>
#include <map>
#include "ai_server/game/agent/base.h"
#include "ai_server/model/world.h"
#include <Eigen/Dense>

namespace ai_server {
namespace game {
namespace agent {

class marking : public base {
private:
  const std::vector<unsigned int> ids_;
  // mark_pairs_ key:our_id value:enemy_id
  std::map<unsigned int, unsigned int> mark_pairs_;
  std::vector<unsigned int> marker_ids_;
  std::vector<unsigned int> enemy_ids_;

public:
  marking(context& ctx, const std::vector<unsigned int>& ids, bool setplay_flag);
  std::vector<std::shared_ptr<action::base>> execute() override;
  bool setplay_flag_;
  void set_setplay_flag(bool setplay_flag);
};

} // namespace agent
} // namespace game
} // namespace ai_server
#endif
