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
  const auto enemy_x = enemy_robot_.x();
  const auto enemy_y = enemy_robot_.y();
  const auto enemy_d = std::hypot(enemy_x, enemy_y);

  auto radius = 0.0;
  auto unk_x  = 0.0;
  auto unk_y  = 0.0;
  auto dir_x  = 0.0;
  auto dir_y  = 0.0;
  auto x      = 0.0;
  auto y      = 0.0;

  switch (mode_) {
    case 0:
      dir_x = unk_x = world_.ball().x();
      dir_y = unk_y = world_.ball().y();
      radius        = 200.0;
      break;
    case 1:
      unk_x  = -4500.0;
      unk_y  = 0.0;
      dir_x  = enemy_x;
      dir_y  = enemy_y;
      radius = 1100.0;
      break;
  }

  const auto ball_d = std::hypot(unk_x, unk_y);

  //ボール(ゴール)と敵機の距離から欲しい距離(ロボット1台分)を計算して、その率を出している
  const auto coefficient = radius / std::hypot(enemy_x - unk_x, enemy_y - unk_y);

  /*ボール(ゴール)と敵機のどちらがより原点より遠くにあるかを判別して、そこから拡大縮小率を掛けて目的の座標を出す*/
  if (ball_d > enemy_d) {
    x = unk_x * coefficient;
    y = unk_y * coefficient;
  } else {
    x = enemy_x * coefficient;
    y = enemy_y * coefficient;
  }

  /*向きをボールの方へ*/
  const auto theta = ai_server::util::wrap_to_2pi(std::atan2(dir_x - x, dir_y - y));
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
