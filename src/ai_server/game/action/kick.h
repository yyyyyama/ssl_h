#ifndef AI_SERVER_GAME_ACTION_KICK_H
#define AI_SERVER_GAME_ACTION_KICK_H

#include "ai_server/model/command.h"
#include "base.h"
#include <boost/math/constants/constants.hpp>
#include <Eigen/Core>

namespace ai_server {
namespace game {
namespace action {

class kick : public base {
public:
  kick(context& ctx, unsigned int id);

  enum class mode { goal, ball };
  enum class running_state { move, round, kick };

  void kick_to(double x, double y);

  void set_kick_type(const model::command::kick_flag_t& kick_type);

  // 蹴れる位置に移動するときに蹴る目標位置を見ているかボールを見ているか指定する関数
  void set_mode(mode mod);
  void set_dribble(int dribble);
  // 目標位置と打つ角度の許容誤差
  void set_angle_margin(double margin);
  running_state state() const;
  void set_stop_ball(bool stop_ball_flag);

  model::command execute() override;

  bool finished() const override;

private:
  mode mode_;
  running_state state_;
  int dribble_;
  double margin_;
  model::command::kick_flag_t kick_type_;
  bool finishflag_;
  bool stop_ball_flag_;
  Eigen::Vector2d target_;
};
} // namespace action
} // namespace game
} // namespace ai_server
#endif
