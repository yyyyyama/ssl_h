#ifndef AI_SERVER_GAME_ACTION_AUTONOMOUS_BALL_PLACE_H
#define AI_SERVER_GAME_ACTION_AUTONOMOUS_BALL_PLACE_H

#include "ai_server/util/time.h"
#include "base.h"

namespace ai_server {
namespace game {
namespace action {

class autonomous_ball_place : public base {
public:
  autonomous_ball_place(const model::world& world, bool is_yellow, unsigned int id,
                        double target_x, double target_y);

  // move:ボール前まで移動
  // hold:ボールを持つ
  // place:ボールを指定位置まで運ぶ
  // wait:ボールの回転を止めるためドリブルバーを止めて停止
  // leave:ボールから離れる
  // finished:動作終了(停止)
  enum class running_state { move, hold, place, wait, leave, finished };

  model::command execute() override;

  running_state state() const;

  bool finished() const override;

private:
  model::command command_;
  running_state state_;
  bool finished_;
  bool wait_flag_;
  bool round_flag_;
  util::time_point_type begin_;
  util::time_point_type now_;
  double target_x_;
  double target_y_;
  double first_ballx_;
  double first_bally_;
};
} // namespace action
} // namespace game
} // namespace ai_server

#endif
