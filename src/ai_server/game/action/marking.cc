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
  auto opponent_x = 0.0;
  auto opponent_y = 0.0;
  auto radius     = 0.0;
  switch (mode_) {
    case 0:
      /*敵機とボールをマーク*/
      opponent_x = world_.ball().x();
      opponent_y = world_.ball().y();
      radius     = 180.0;
      break;
    case 1:
      /*敵機のシュートをマーク
       * 座標>0が敵陣地			*/
      opponent_x = -4500.0;
      opponent_y = 0.0;
      radius     = 1000.0;
      break;
  }
  const auto enemy_x = enemy_robot_.x();
  const auto enemy_y = enemy_robot_.y();
  /*計算過程で使用する定数と係数*/
  const auto constant_1 =
      (-opponent_x * (enemy_y - opponent_x) + opponent_y * (enemy_x - opponent_x)) /
      (enemy_x - enemy_x - opponent_x);
  const auto constant_2 = (enemy_y - opponent_y) / (enemy_x - opponent_x);

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
  const auto theta = ai_server::util::wrap_to_2pi(std::atan2(opponent_x - x, opponent_y - y));
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
