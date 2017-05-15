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
int guard::dribble() {
  return dribble_;
}
model::command guard::execute() {
  //それぞれ自機を生成
  model::command command(id_);

  //キックフラグを立てて置く
  command.set_kick_flag(kick_type_);

  command.set_dribble(dribble_);

  flag_ = false;

  const auto robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  if (!robots.count(id_)) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  const auto robot = robots.at(id_);
  const Eigen::Vector2d robot_pos{robot.x(), robot.y()};
  const auto robot_theta = util::wrap_to_pi(robot.theta());
  const auto omega       = theta_ - robot_theta;
  //速度の係数
  auto c = 10.0;
  if (((pos_ - robot_pos) * c).norm() > 1400.0) {
    c = 5.0;
  }
  const Eigen::Vector2d vec{(pos_ - robot_pos) * c};

  //位置のマージン
  // const auto margin = 0.1 * vec.norm();
  auto margin = 80.0;
  if (vec.norm() < 1400) {
    margin = 10.0;
  }
  //目標位置に居るなら終わり
  if (std::pow(robot_pos.x() - pos_.x(), 2) + std::pow(robot_pos.y() - pos_.y(), 2) -
          std::pow(margin, 2) <
      0) {
    command.set_velocity({0.0, 0.0, omega});
    flag_ = true;
    return command;
  }

  command.set_velocity({vec.x(), vec.y(), omega});

  return command;
}
bool guard::finished() const {
  return flag_;
}
}
}
}
