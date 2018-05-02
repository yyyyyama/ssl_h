#ifndef AI_SERVER_GAME_AGENT_PENALTY_KICK_H
#define AI_SERVER_GAME_AGENT_PENALTY_KICK_H

#include "base.h"
#include "ai_server/model/ball.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/no_operation.h"
#include "ai_server/game/action/rush.h"
#include "ai_server/game/action/kick_action.h"

#include <memory>
#include <vector>

namespace ai_server {
namespace game {
namespace agent {

class penalty_kick : public base {
public:
  penalty_kick(const model::world& world, bool is_yellow, unsigned int kicker_id,
               const std::vector<unsigned int>& ids, unsigned int enemy_keeper);
  enum class penalty_mode { attack, defense }; //攻撃側と守備側で変える

  penalty_kick::penalty_mode mode();
  void set_mode(penalty_kick::penalty_mode);
  bool start_flag() const;
  void set_start_flag(bool start_flag);
  std::vector<std::shared_ptr<action::base>> execute() override;
  bool finished();

private:
  penalty_mode mode_;
  //パラメータ計算
  void calculate_kick_position(double keep_out); //半径keep_outだけボールに近づく
  bool start_rush(model::ball ball);
  bool time_over(std::chrono::high_resolution_clock::time_point point, int count);

  bool start_flag_;
  std::vector<unsigned int> ids_;
  std::chrono::high_resolution_clock::time_point change_command_time_;
  std::shared_ptr<action::move> move_;
  std::shared_ptr<action::rush> rush_;
  std::shared_ptr<action::move> rush_move_;

  bool initial_flag_;
  int shoot_count_;
  model::ball setted_ball_;
  model::robot setted_robot_;

  unsigned int kicker_id_;
  // PK待機位置
  double kick_x_;
  double kick_y_;
  double kick_theta_;
  double theta_;
  double prev_theta_;
  bool prev_dec_;

  //敵キーパーの動きを監視して蹴るタイミングを変える
  unsigned int keeper_id_;
};
} // namespace agent
} // namespace game
} // namespace ai_server

#endif
