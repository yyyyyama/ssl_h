#include <cmath>

#include "ai_server/game/action/marking.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace action {
void marking::mark_robot(unsigned int enemy_id) {
  enemy_id_ = enemy_id;
}
void marking::mode_choose(mark_mode mode) {
  switch (mode) {
		case mark_mode::pass_block:  //ボールを受け取るのを阻止(内分点)
		case mark_mode::kick_block:  //ボールを蹴るのを阻止(外分点)
		case mark_mode::shoot_block: //シュートを阻止
      mode_ = mode;
      break;
    default:
      break;
  }
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
  auto ratio         = 0.0; //敵位置とボールの比
  auto tmp           = 0.0; //敵位置 - 自位置の比
  switch (mode_) {
		case mark_mode::pass_block: //ボールを受け取るのを阻止(内分点)
      ratio = 200.0 / std::hypot(enemy_x - ball_x, enemy_y - ball_y);
      x     = (1 - ratio) * enemy_x + ratio * ball_x;
      y     = (1 - ratio) * enemy_y + ratio * ball_y;
      tmp_x = ball_x;
      tmp_y = ball_y;
      break;
		case mark_mode::kick_block: //ボールを蹴るのを阻止(外分点)
      tmp   = std::hypot(enemy_x - ball_x, enemy_y - ball_y) / 300.0;
      ratio = 1 - tmp;
      x     = (-ratio * enemy_x + ball_x) / tmp;
      y     = (-ratio * enemy_y + ball_y) / tmp;
      tmp_x = ball_x;
      tmp_y = ball_y;
      break;
		case mark_mode::shoot_block: //シュートを阻止
      ratio = 1500.0 / std::hypot(4500 - enemy_x, enemy_y);
      x     = (1 - ratio) * 4500 + ratio * enemy_x;
      y     = ratio * enemy_y;
      tmp_x = enemy_x;
      tmp_y = enemy_y;
  }

  //向きをボールの方へ
  const auto theta = util::wrap_to_2pi(std::atan2(y - tmp_y, x - tmp_x) + pi<double>());

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
