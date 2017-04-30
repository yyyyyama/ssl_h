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
  enum class mode { move_to_ball, wraparound, dribble } mode_ = mode::move_to_ball;
  void set_target(double x, double y);
  model::command execute() override;
  bool finished() const override;

private:
  double target_x_;
  double target_y_;

  double first_dist_; // 開始時のfirst_posまでの距離

  int count_     = 0; // 移動時間のカウント
  int sub_count_ = 0; // 減速時間のカウント

  bool init_flag_ = false;
  bool wrap_flag_ = false;
  bool sign_flag_ = false;
  bool fin_flag_  = false;
};

} // namespace ai_server
} // namespace game
} // namespace action

#endif // AI_SERVER_GAME_ACTION_CHASE_BALL_H
