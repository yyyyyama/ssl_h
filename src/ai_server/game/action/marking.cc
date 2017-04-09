#include <cmath>

#include "ai_server/game/action/marking.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace action {
void marking::mark_robot(unsigned int enemy_id) {
  enemy_id_ = enemy_id;
}
void marking::mark_mode(unsigned int mode) {
  mode_ = mode;
}
model::command marking::execute() {
  using boost::math::constants::pi;

  //それぞれ自機と敵機を生成
  model::command ally_robot(id_);
  const auto enemy_robots  = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  const auto& enemy_robot_ = enemy_robots.at(enemy_id_);
  //必要なパラメータ
  const auto enemy_x = enemy_robot_.x();
  const auto enemy_y = enemy_robot_.y();
  const auto ball_x  = world_.ball().x();
  const auto ball_y  = world_.ball().y();
  auto tmp_x         = 0.0;
  auto tmp_y         = 0.0;
  auto x             = 0.0;
  auto y             = 0.0;
  auto m             = 0.0;
  auto tmp           = 0.0;
  switch (mode_) {
    case 0: //ボールを受け取るのを阻止(内分点)
      m     = 200.0 / std::hypot(enemy_x - ball_x, enemy_y - ball_y);
      x     = (1 - m) * enemy_x + m * ball_x;
      y     = (1 - m) * enemy_y + m * ball_y;
      tmp_x = ball_x;
      tmp_y = ball_y;
      break;
    case 1: //ボールを蹴るのを阻止(外分点)
      tmp   = std::hypot(enemy_x - ball_x, enemy_y - ball_y) / 300.0;
      m     = 1 - tmp;
      x     = (-m * enemy_x + ball_x) / tmp;
      y     = (-m * enemy_y + ball_y) / tmp;
      tmp_x = ball_x;
      tmp_y = ball_y;
      break;
    case 2: //シュートを阻止
      m     = 1500.0 / std::hypot(4500 - enemy_x, enemy_y);
      x     = (1 - m) * 4500 + m * enemy_x;
      y     = m * enemy_y;
      tmp_x = enemy_x;
      tmp_y = enemy_y;
  }

  //向きをボールの方へ
  const auto theta =
      ai_server::util::wrap_to_2pi(std::atan2(y - tmp_y, x - tmp_x) + pi<double>());

  //計算した値を自機にセット
  ally_robot.set_position({x, y, theta});

  return ally_robot;
}
bool marking::finished() const {
  return flag;
}
}
}
}
