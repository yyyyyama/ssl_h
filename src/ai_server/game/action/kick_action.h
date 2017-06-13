#ifndef AI_SERVER_GAME_ACTION_KICK_ACTION_H
#define AI_SERVER_GAME_ACTION_KICK_ACTION_H

#include "ai_server/model/command.h"
#include "base.h"
#include <boost/math/constants/constants.hpp>

namespace ai_server {
namespace game {
namespace action {

class kick_action : public base {
public:
  using base::base;

  enum class mode { goal, ball } mode_ = mode::goal;

  void kick_to(double x, double y);

  void set_kick_type(const model::command::kick_flag_t& kick_type);

  // 蹴れる位置に移動するときに蹴る目標位置を見ているかボールを見ているか指定する関数
  void set_mode(mode mod);
  void set_dribble(int dribble);
  // 目標位置と打つ角度の許容誤差
  void set_angle_margin(double margin);

  model::command execute() override;

  bool finished() const override;

private:
  double x_;
  double y_;
  double old_ball_x;
  double old_ball_y;
  int dribble_   = 3;
  double margin_ = 0.07;
  model::command::kick_flag_t kick_type_;
  bool finishflag_  = false;
  bool aroundflag_  = false;
  bool advanceflag_ = false;
};
} // namespace action
} // namespace game
} // namespace ai_server
#endif
