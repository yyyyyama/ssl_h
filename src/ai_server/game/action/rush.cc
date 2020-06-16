#include "ai_server/game/action/rush.h"

#include <cmath>

namespace ai_server {
namespace game {
namespace action {

rush::rush(context& ctx, unsigned int id)
    : base(ctx, id),
      kick_type_({model::command::kick_type_t::none, 0.0}),
      flag_(false),
      previous_kick_ball_(world().ball()) {}

model::command rush::execute() {
  model::command command{};

  const auto robots = model::our_robots(world(), team_color());
  const auto ball   = world().ball();

  //見えなかったら止める
  if (!robots.count(id_)) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  //速度計算
  const auto& robot = robots.at(id_);
  double theta      = std::atan2(ball.y() - robot.y(), ball.x() - robot.x());

  //ボールを蹴ったと判断したら終了
  if (std::hypot(previous_kick_ball_.x() - ball.x(), previous_kick_ball_.y() - ball.y()) > 50) {
    command.set_velocity({0.0, 0.0, 0.0});
    flag_ = true;
    return command;
  }

  //ボール位置で動作させる
  command.set_kick_flag({model::command::kick_type_t::line, 60});
  command.set_position({ball.x(), ball.y(), theta});

  flag_               = false;
  previous_kick_ball_ = ball;
  return command;
}

bool rush::finished() const {
  return flag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
