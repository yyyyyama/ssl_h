#ifndef AI_SERVER_GAME_AGENT_PENALTY_KICK_H
#define AI_SERVER_GAME_AGENT_PENALTY_KICK_H

#include "base.h"
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
               const std::vector<unsigned int>& ids);
  enum class penalty_mode { attack, defense }; //攻撃側と守備側で変える

  //敵の動きを見て蹴り方を変える
  // normal: 敵のいない方向に蹴る
  // rush:   敵が目の前にいないときに蹴る
  enum class kicker_type { normal, rush };

  penalty_kick::penalty_mode mode();
  void set_mode(penalty_kick::penalty_mode);
  bool start_flag() const;
  void set_start_flag(bool start_flag);
  std::vector<std::shared_ptr<action::base>> execute() override;
  bool finished();

private:
  penalty_mode mode_;
  kicker_type type_;
  //パラメータ計算
  void calculate_kick_position(double keep_out); //半径keep_outだけボールに近づく
  void calculate_target();                       //蹴る目標を決定する
  //敵キーパーの動作を監視
  void watch_keeper();

  bool start_flag_;
  std::vector<unsigned int> ids_;

  std::shared_ptr<action::move> move_;
  std::shared_ptr<action::kick_action> kick_;
  std::shared_ptr<action::rush> rush_;
  std::shared_ptr<action::move> rush_move_;

  unsigned int kicker_id_;
  // PK待機位置
  double kick_x_;
  double kick_y_;
  double kick_theta_;

  //蹴る方向
  double target_x_;
  double target_y_;

  //敵キーパーの動きを監視して蹴るタイミングを変える
  unsigned int keeper_id_;
  unsigned int
      keeper_move_count_; // past_keeper_y_max_とpast_keeper_y_minを比較し,差が大きいときにカウント
  std::vector<double> past_keeper_y_; //過去のキーパーのy座標,1往復すると初期化される

  model::ball prev_ball_;
};
}
}
}

#endif