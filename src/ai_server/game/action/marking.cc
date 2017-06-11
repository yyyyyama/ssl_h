#include <cmath>
#include <Eigen/Dense>

#include "ai_server/game/action/marking.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace action {
unsigned int marking::id() {
  return id_;
}
void marking::mark_robot(unsigned int enemy_id) {
  enemy_id_ = enemy_id;
}
void marking::set_mode(mark_mode mode) {
  mode_ = mode;
}
void marking::set_radius(double radius) {
  radius_ = radius;
}
model::command marking::execute() {
  using boost::math::constants::pi;

  //それぞれ自機と敵機を生成
  model::command ally_robot(id_);
  const auto enemy_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  //指定されたロボットが見えなかったらその位置で停止
  if (!enemy_robots.count(enemy_id_)) {
    ally_robot.set_velocity({0.0, 0.0, 0.0});
    return ally_robot;
  }
  const auto& enemy_robot = enemy_robots.at(enemy_id_);
  //必要なパラメータ
  const Eigen::Vector2d enemy{enemy_robot.x(), enemy_robot.y()};
  const Eigen::Vector2d ball{world_.ball().x(), world_.ball().y()};
  const Eigen::Vector2d goal{world_.field().x_min(), 0.0};
  Eigen::Vector2d tmp_pos{0.0, 0.0};
  Eigen::Vector2d position{0.0, 0.0};
  auto ratio = 0.0; //敵位置とボールの比
  auto tmp   = 0.0; //どうでもいい一時的な数値に使う
  switch (mode_) {
    case mark_mode::kick_block:                   //ボールを蹴るのを阻止(外分点)
      tmp      = (enemy - ball).norm() / radius_; //敵位置 - 自位置の比
      ratio    = 1 - tmp;
      position = (-ratio * enemy + ball) / tmp;
      tmp_pos  = ball;
      break;
    case mark_mode::shoot_block:                 //シュートを阻止
      const auto length = (goal - enemy).norm(); //敵機とゴールの距離
      tmp               = std::signbit(1400 - length / 2)
                ? 0
                : 1400 - length / 2; //敵機-ゴール中央の中間地点とゴールラインの差
      ratio    = (length / 2 + tmp) / length;
      position = (1 - ratio) * goal + ratio * enemy;
      tmp_pos  = enemy;
  }

  //向きをボールの方へ
  const auto theta = util::wrap_to_2pi(
      std::atan2(position.y() - tmp_pos.y(), position.x() - tmp_pos.x()) + pi<double>());

  //計算した値を自機にセット
  ally_robot.set_position({position.x(), position.y(), theta});

  return ally_robot;
}
bool marking::finished() const {
  return flag;
}
}
}
}
