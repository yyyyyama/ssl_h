#include <cmath>

#include "ai_server/game/action/receive.h"
#include "ai_server/util/math/to_vector.h"
#include "ai_server/util/math.h"

#include <iostream>

namespace ai_server {
namespace game {
namespace action {
receive::receive(const model::world& world, bool is_yellow, unsigned int id)
    : base(world, is_yellow, id) {
  const auto& robots   = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto& robot    = robots.at(id_);
  const auto robot_pos = util::math::position(robot);
  dummy_pos_           = robot_pos;
}
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
void receive::set_shoot(Eigen::Vector2d shoot_pos) {
  shoot_pos_  = shoot_pos;
  shoot_flag_ = true;
}
void receive::set_kick_type(const model::command::kick_flag_t& kick_type) {
  kick_type_ = kick_type;
}
model::command receive::execute() {
  //それぞれ自機を生成
  model::command command(id_);

  const auto& robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  if (!robots.count(id_) || !robots.count(passer_id_)) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  const auto& robot = robots.at(id_);
  const auto robot_theta = util::wrap_to_pi(robot.theta());
  const auto robot_pos =
      util::math::position(robot) +
      (shoot_flag_ ? 80 * Eigen::Vector2d(std::cos(robot_theta), std::sin(robot_theta))
                   : Eigen::Vector2d(0, 0));
  const auto ball_pos = util::math::position(world_.ball());
  const auto ball_vec = util::math::velocity(world_.ball());

  const auto& passer      = robots.at(passer_id_);
  const auto passer_pos   = util::math::position(passer);
  const auto passer_theta = util::wrap_to_pi(passer.theta());

  //ボールがめっちゃ近くに来たら受け取ったと判定
  //現状だとボールセンサに反応があるか分からないので
  if (shoot_flag_) {
    if ((robot_pos - ball_pos).norm() < 100) {
      approaching_flag_ = true;
    } else if (approaching_flag_) {
      flag_ = true;
      command.set_velocity({0.0, 0.0, 0.0});
      return command;
    }
  } else if ((robot_pos - ball_pos).norm() < 100) {
    flag_ = true;
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  decltype(util::math::position(passer)) normalize; //正面に移動したい対象の単位ベクトル
  decltype(util::math::position(passer)) position; //正面に移動したい対象の位置

  if (ball_vec.norm() < 500) { // passerの正面に移動したい
    normalize = Eigen::Vector2d{std::cos(passer_theta), std::sin(passer_theta)};
    position  = passer_pos;
  } else { //ボールの移動予測地点に移動したい
    normalize = ball_vec.normalized();
    position  = ball_pos;
  }

  //対象とreceiverの距離
  const auto length = robot_pos - position;
  //内積より,対象と自分の直交する位置
  const auto dot = normalize.dot(length);

  //目標位置と角度
  const auto to_ball  = ball_pos - robot_pos;
  const auto to_shoot = shoot_pos_ - robot_pos;
  std::cout << "shoot_pos_<<" << shoot_pos_.x() << ", " << shoot_pos_.y() << std::endl;
  const auto theta = shoot_flag_ ? std::atan2(to_shoot.y(), to_shoot.x())
                                 : std::atan2(to_ball.y(), to_ball.x());
  const auto target = (position + dot * normalize*0.95) +
                      (shoot_flag_ ? -80 * Eigen::Vector2d(std::cos(theta), std::sin(theta))
                                   : Eigen::Vector2d(0, 0));

  std::cout<<"shoot_flag_"<<shoot_flag_<<std::endl;
  //位置から速度へ
  Eigen::Vector2d target_vec{(target - robot_pos) * 8};
  std::cout << "target__vec" << target_vec.x() << ", " << target_vec.y() << std::endl;
  if ((robot_pos - ball_pos).norm() < 300 && ball_vec.norm() > 500) {
    target_vec = target_vec + ball_vec / 3;
  }
  std::cout << "target_vec" << target_vec.x() << ", " << target_vec.y() << std::endl;

  const auto omega = (theta - robot_theta);
  command.set_position({target.x(), target.y(), theta});
  if (shoot_flag_) {
    command.set_kick_flag(kick_type_);
  } else {
    //ドリブルさせる
    command.set_dribble(dribble_);
  }

  flag_ = false;
  return command;
}
bool receive::finished() const {
  return flag_;
}
}
}
}
