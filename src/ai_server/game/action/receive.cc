#include <cmath>

#include "ai_server/game/action/receive.h"
#include "ai_server/util/math/to_vector.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace action {
void receive::set_dribble(int dribble) {
  dribble_ = dribble;
}
int receive::dribble() {
  return dribble_;
}
void receive::set_passer(unsigned int passer_id) {
  passer_id_ = passer_id;
}
unsigned int receive::passer() {
  return passer_id_;
}
model::command receive::execute() {
  //それぞれ自機を生成
  model::command command(id_);

  //ドリブルさせる
  command.set_dribble(dribble_);

  const auto& robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  if (!robots.count(id_) || !robots.count(passer_id_)) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  const auto& robot      = robots.at(id_);
  const auto robot_pos   = util::math::position(robot);
  const auto robot_theta = util::wrap_to_pi(robot.theta());
  const auto ball_pos    = util::math::position(world_.ball());
  const auto ball_vec    = util::math::velocity(world_.ball());

  const auto& passer    = robots.at(passer_id_);
  const auto passer_pos = util::math::position(passer);
  const auto passer_theta = util::wrap_to_pi(passer.theta());

  //ボールがめっちゃ近くに来たら受け取ったと判定
  //現状だとボールセンサに反応があるか分からないので
  if ((robot_pos - ball_pos).norm() < 120) {
    flag_ = true;
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  decltype(util::math::position(passer)) tmp;
  if (ball_vec.norm() < 0.5) {
    tmp = passer_pos + Eigen::Vector2d{std::cos(passer_theta),std::sin(passer_theta)};
  } else {
    tmp = ball_vec;
  }

  const auto length    = robot_pos - ball_pos;
  const auto normalize = tmp.normalized();
  const auto dot       = normalize.dot(length);
  //目標位置と角度
  const auto target = (ball_pos + dot * normalize);
  const auto theta  = std::atan2(-normalize.y(), -normalize.x());

  command.set_position({target.x(), target.y(), theta});

  flag_ = false;
  return command;
}
bool receive::finished() const {
  return flag_;
}
}
}
}
