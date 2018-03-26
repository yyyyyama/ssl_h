#include <cmath>

#include "ai_server/game/action/guard.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace action {
void guard::move_to(double x, double y, double theta) {
  pos_.x() = x;
  pos_.y() = y;
  theta_   = theta;
}
model::command::kick_flag_t guard::kick_type() {
  return kick_type_;
}
void guard::set_kick_type(const model::command::kick_flag_t& kick_type) {
  kick_type_ = kick_type;
}
void guard::set_dribble(int dribble) {
  dribble_ = dribble;
}
void guard::set_halt(bool halt_flag) {
  halt_flag_ = halt_flag;
}
void guard::set_magnification(double magnification) {
  magnification_ = magnification;
}
int guard::dribble() {
  return dribble_;
}
unsigned int guard::id() {
  return id_;
}
model::command guard::execute() {
  //それぞれ自機を生成
  model::command command(id_);

  //キックフラグを立てて置く
  command.set_kick_flag(kick_type_);

  command.set_dribble(dribble_);

  const auto robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  if (!robots.count(id_) || halt_flag_) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  const auto robot = robots.at(id_);
  const Eigen::Vector2d robot_pos{robot.x(), robot.y()};
  const auto robot_theta = util::wrap_to_pi(robot.theta());
  const auto omega       = theta_ - robot_theta;
  if ((robot_pos - pos_).norm() < 100.0) {
    magnification_ = 1000.0;
  }

  const Eigen::Vector2d vec{(pos_ - robot_pos).normalized() * magnification_};

  //位置のマージン

  const auto margin = 0.0;

  //目標位置に居るなら終わり
  //目標位置から現在地の距離
  const auto lengh = (robot_pos - pos_).norm();
  if (lengh < margin) {
    command.set_velocity({0.0, 0.0, omega});
    flag_ = true;
    return command;
  }

  command.set_position({pos_.x(), pos_.y(), theta_});

  flag_ = false;
  return command;
}
bool guard::finished() const {
  return flag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
