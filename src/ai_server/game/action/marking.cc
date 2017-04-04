#include <cmath>

#include "ai_server/game/action/marking.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace action {
void marking::mark_robot(unsigned int enemy_id) {
  enemy_id_ = enemy_id;
}
void marking::mark_mode(unsigned int mode){
	mode_ = mode;
}
model::command marking::execute() {
  //それぞれ自機と敵機を生成
  model::command ally_robot(id_);
  const auto enemy_robots  = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  const auto& enemy_robot_ = enemy_robots.at(enemy_id_);
  /*必要なパラメータ*/
  const auto ball_x  = world_.ball().x();
  const auto ball_y  = world_.ball().y();
  const auto enemy_x = enemy_robot_.x();
  const auto enemy_y = enemy_robot_.y();
  const auto radius  = 180;
  /*計算過程で使用する定数と係数*/
  const auto constant_1 = (-ball_x * (enemy_y - ball_x) + ball_y * (enemy_x - ball_x)) /
                          (enemy_x - enemy_x - ball_x);
  const auto constant_2 = (enemy_y - ball_y) / (enemy_x - ball_x);

  const auto coefficient_1 = 1 + std::pow(constant_2, 2);
  const auto coefficient_2 =
      -enemy_x / 2 + (constant_2 / 2) * constant_1 - (enemy_y / 2) * constant_2;
  const auto coefficient_3 =
      (std::pow(radius, 2) - std::pow(enemy_x, 2) - std::pow(enemy_y, 2)) +
      (enemy_y / 2) * constant_1 - std::pow(constant_1, 2);
  /*解の公式より交点を求める*/
  const auto x = (-coefficient_2 + std::sqrt(std::pow(coefficient_2, 2) -
                                             4 * coefficient_1 * (-coefficient_3))) /
                 (2 * coefficient_1);
  const auto y = x * constant_2 + constant_1;
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
