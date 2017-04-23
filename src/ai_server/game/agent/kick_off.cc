#include <cmath>
#include "kick_off.h"
#include "ai_server/game/action/kick_action.h"
#include "ai_server/game/action/no_operation.h"
#include "ai_server/model/command.h"

namespace ai_server {
namespace game {
namespace agent {

kick_off::kick_off(const model::world& world, bool is_yellow, unsigned int kicker_id)
    : base(world, is_yellow), kicker_id_(kicker_id) {}
kick_off::set_start_flag(bool start_flag) {
  start_flag_ = start_flag;
}
kick_off::start_flag() const {
  return start_flag_;
}
kick_off::execute() {
  const double robot_r    = 85.0;  //ロボットの半径
  const double keep_out_r = 500.0; //キックオフ時の立ち入り禁止区域の半径

  if (kick_finished_) {
    //ボールを蹴り終わった時
    game::action::no_operation no_op{world_, is_yellow_, kicker_id_};
    no_op.execute();
    return no_op;
  } else {
    if (move_finished_ && start_flag_) {
      // StartGameが指定され、所定の位置に移動済みの時
      game::action::kick_action kick{world_, is_yellow_, kicker_id_};
      kick.kick_to(world_.field().x_max(), 0.0);
      kick.set_kick_type(static_cast<int>(model::command::kick_type_t::backspin));
      kick.set_mode(static_cast<int>(game::action::kick_action::mode::goal));
      kick.execute();

      kick_finished_ = kick.finished();
      return kick;

    } else {
      // StartGameが指定されていない、または所定の位置に移動していない時
      const auto ball = world_.ball();

      //ボールとゴールを結んだライン上で、ボールから半径keep_out_r+ロボット半径離れたところに移動
      ball_goal_theta_ = atan(-ball.y() / (world_.field().x_max() - ball.x()));
      move_to_x_       = ball.x() - (keep_out_r + robot_r) * cos(ball_goal_theta_);
      move_to_y_       = ball.y() - (keep_out_r + robot_r) * sin(ball_goal_theta_);
      move_to_theta_   = ball_goal_theta_;

      game::action::move move{world_, is_yellow_, kicker_id_};
      move.move_to{move_to_x_, move_to_y_, move_to_theta_};
      move.execute();

      move_finished_ = move.finished();
      return move;
    }
  }
}

} // namespace agent
} // namespace agent
} // napespace ai_server
