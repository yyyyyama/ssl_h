#ifndef AI_SERVER_GAME_ACTION_CHASE_BALL_H
#define AI_SERVER_GAME_ACTION_CHASE_BALL_H

#include "ai_server/model/command.h"
#include "base.h"

namespace ai_server {
namespace game {
namespace action {

class chase_ball : public base {
public:
  using base::base;
  enum class mode { move_to_ball, wraparound, dribble, wait_ball } mode_ = mode::move_to_ball;
  void set_target(double x, double y);
  model::command execute() override;
  bool finished() const override;

private:
  double kick_target_x_; // 蹴りたい方向
  double kick_target_y_;

  double start_dist_; // mode::wait_ball 開始時のfirst_posまでの距離

  int count_     = 0; // 移動時間のカウント
  int sub_count_ = 0; // 減速時間のカウント

  double ball_x_, ball_y_;   // mode::wait_ball 開始時のballの位置
  double ball_vx_, ball_vy_; // ballの速度(速度フィルター)

  bool init_flag_ = false; //初期化
  bool wait_flag_ = false; //ボールを待つ動作の判断
  bool wrap_flag_ = false; //回り込みの判断
  bool sign_flag_ = false; //回り込みの正負の判断
  bool fin_flag_  = false; //終了
};

} // namespace ai_server
} // namespace game
} // namespace action

#endif // AI_SERVER_GAME_ACTION_CHASE_BALL_H
