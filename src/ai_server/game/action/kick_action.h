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
  kick_action(const model::world& world, bool is_yellow, unsigned int id);

  enum class mode { goal, ball };
  enum class running_state { move, kick, finished };

  void kick_to(double x, double y);

  void set_kick_type(const model::command::kick_flag_t& kick_type);

  // 蹴れる位置に移動するときに蹴る目標位置を見ているかボールを見ているか指定する関数
  void set_mode(mode mod);
  void set_dribble(int dribble);
  // 目標位置と打つ角度の許容誤差
  void set_angle_margin(double margin);
  running_state state() const;

  model::command execute() override;

  bool finished() const override;

private:
  mode mode_;
  running_state state_;
  double x_;
  double y_;
  int dribble_;
  double margin_;
  model::command::kick_flag_t kick_type_;
  bool finishflag_;
  bool aroundflag_;
  bool advanceflag_;
  double old_ball_x_;
  double old_ball_y_;
};
} // namespace action
} // namespace game
} // namespace ai_server
#endif
