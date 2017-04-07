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
  //それぞれ自機と敵機を生成
  model::command ally_robot(id_);
  const auto enemy_robots  = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  const auto& enemy_robot_ = enemy_robots.at(enemy_id_);
  /*必要なパラメータ*/
  const auto radius  = 180;
  const auto enemy_x = enemy_robot_.x();
  const auto enemy_y = enemy_robot_.y();
  const auto enemy_d = std::hypot(enemy_x, enemy_y);
  const auto ball_x  = world_.ball().x();
  const auto ball_y  = world_.ball().y();
  const auto ball_d  = std::hypot(ball_x, ball_y);

  const auto coefficient = radius / std::hypot(enemy_x - ball_x, enemy_y - ball_y);

  auto x = 0.0;
  auto y = 0.0;

  if (ball_d > enemy_d) {
    x = ball_x * coefficient + radius / 2;
    y = ball_y * coefficient + radius / 2;
  } else {
    x = enemy_x * coefficient + radius / 2;
    y = enemy_y * coefficient + radius / 2;
  }

  /*向きをボールの方へ*/
  const auto theta = ai_server::util::wrap_to_2pi(std::atan2(ball_x - x, ball_y - y));
  /*計算した値を自機にセット*/
  ally_robot.set_position({x, y, theta});

  return ally_robot;
}
bool marking::finished() const {
  return flag;
}
}
}
}
