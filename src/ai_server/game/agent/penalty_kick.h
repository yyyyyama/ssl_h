#ifndef AI_SERVER_GAME_AGENT_PENALTY_KICK_H
#define AI_SERVER_GAME_AGENT_PENALTY_KICK_H

#include "base.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/turn_kick.h"

#include <memory>
#include <vector>

namespace ai_server {
namespace game {
namespace agent {

class penalty_kick : public base {
public:
  penalty_kick(context& ctx, unsigned int kicker_id, const std::vector<unsigned int>& ids,
               unsigned int enemy_keeper);
  enum class penalty_mode { attack, defense }; //攻撃側と守備側で変える

  penalty_kick::penalty_mode mode() const;
  void set_mode(penalty_kick::penalty_mode);
  bool start_flag() const;
  void set_start_flag(bool start_flag);
  std::vector<std::shared_ptr<action::base>> execute() override;
  bool finished();

private:
  enum class attack_state { wait, check, kick }; //攻撃の状態
  attack_state state_;

  penalty_mode mode_;

  bool start_flag_;
  std::vector<unsigned int> ids_;
  std::shared_ptr<action::turn_kick> turn_kick_;
  std::shared_ptr<action::move> kicker_move_;

  unsigned int kicker_id_;

  //敵キーパーの動きを監視して蹴るタイミングを変える
  unsigned int keeper_id_;

  std::chrono::steady_clock::time_point change_command_time_; // PKの経過時間
};
} // namespace agent
} // namespace game
} // namespace ai_server

#endif
