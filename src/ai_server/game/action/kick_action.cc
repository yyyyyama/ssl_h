#include <cmath>
#include "kick_action.h"

namespace ai_server {
namespace game {
namespace action {

void kick_action::kick_to(double x, double y) {
  x_ = x;
  y_ = y;
}
void kick_action::set_kick_type(const model::command::kick_flag_t& kick_type) {
  kick_type_ = kick_type;
}
model::command kick_action::execute() {
  model::command command_(id_);
  if (count_ == 0) {
    exeflag_ = false;
    // ロボット、ボール、蹴りたい位置となるように移動
    const auto ball = world_.ball();
    ball_x_         = ball.x();
    ball_y_         = ball.y();
    dx_             = x_ - ball_x_;
    dy_             = y_ - ball_y_;
    const double r  = std::hypot(dx_, dy_);
    // ボールとロボットの間は15cm（適当）
    x_                    = (-dx_ / r) * 150 + ball_x_;
    y_                    = (-dy_ / r) * 150 + ball_y_;
    const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
    const auto& robot_me  = our_robots.at(id_);
    robot_pos_            = {x_, y_, robot_me.theta()};

    command_.set_position(robot_pos_);
    count_ = 1;
  } else if (count_ == 1) {
    // 位置をそのままにロボットがボールをけれる向きにする
    robot_pos_.theta = std::atan2(dx_, dy_);
    command_.set_position(robot_pos_);
    count_ = 2;
  } else {
    // キックフラグをセットし、ボールの位置まで移動
    robot_pos_ = {ball_x_, ball_y_, robot_pos_.theta};
    command_.set_position(robot_pos_);
    command_.set_kick_flag(kick_type_);
    count_   = 0;
    exeflag_ = true;
  }
  return command_;
};
bool kick_action::finished() const {
  return exeflag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
