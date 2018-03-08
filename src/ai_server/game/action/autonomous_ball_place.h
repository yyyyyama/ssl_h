#ifndef AI_SERVER_GAME_ACTION_AUTONOMOUS_BALL_PLACE_H
#define AI_SERVER_GAME_ACTION_AUTONOMOUS_BALL_PLACE_H

#include <chrono>
#include "base.h"

namespace ai_server {
namespace game {
namespace action {

class autonomous_ball_place : public base {
public:
  autonomous_ball_place(const model::world& world, bool is_yellow, unsigned int id);

  void place_to(double target_x, double target_y);

  enum class running_state { move, hold, place, wait, leave, finished };

  model::command execute() override;

  bool can_leave();

  running_state state() const;

  bool finished() const override;

private:
  model::command command_;
  running_state state_ = running_state::move;
  bool finished_       = false;
  bool wait_flag_      = true;
  bool round_flag_     = false;
  std::chrono::system_clock::time_point begin_;
  std::chrono::system_clock::time_point now_;
  double target_x_;
  double target_y_;
  double first_ballx_;
  double first_bally_;
};
} // namespace action
} // namespace game
} // namespace ai_server

#endif
