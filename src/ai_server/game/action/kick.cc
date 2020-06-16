#include <boost/math/constants/constants.hpp>
#include <cmath>
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"
#include "kick.h"

namespace ai_server {
namespace game {
namespace action {
kick::kick(context& ctx, unsigned int id)
    : base(ctx, id),
      mode_(mode::goal),
      state_(running_state::move),
      dribble_(0),
      margin_(0.05),
      finishflag_(false),
      stop_ball_flag_(false) {}

void kick::kick_to(double x, double y) {
  target_ = Eigen::Vector2d{x, y};
}

void kick::set_kick_type(const model::command::kick_flag_t& kick_type) {
  kick_type_ = kick_type;
}

// 蹴れる位置に移動するときに蹴る目標位置を見ているかボールを見ているか指定する関数
void kick::set_mode(mode mod) {
  mode_ = mod;
}

void kick::set_dribble(int dribble) {
  dribble_ = dribble;
}

void kick::set_angle_margin(double margin) {
  margin_ = margin;
}

kick::running_state kick::state() const {
  return state_;
}

void kick::set_stop_ball(bool stop_ball_flag) {
  stop_ball_flag_ = stop_ball_flag;
}

model::command kick::execute() {
  using boost::math::constants::pi;
  using boost::math::constants::two_pi;

  const auto our_robots           = model::our_robots(world(), team_color());
  const auto& robot_me            = our_robots.at(id_);
  const Eigen::Vector2d robot_pos = util::math::position(robot_me);
  const Eigen::Vector2d ball_pos  = util::math::position(world().ball());
  const Eigen::Vector2d ball_vel  = util::math::velocity(world().ball());
  //ボールから目標
  const double ball_target = std::atan2(target_.y() - ball_pos.y(), target_.x() - ball_pos.x());
  //ボールからロボット
  const double ball_robot =
      std::atan2(robot_pos.y() - ball_pos.y(), robot_pos.x() - ball_pos.x());
  //ロボットからボール
  const double robot_ball = ball_robot + pi<double>();
  //ボールとロボットの間の距離
  const double dist = 350;
  //送りたいロボットを指定
  model::command command{};

  // executeが呼ばれる間にボールがこれだけ移動したら蹴ったと判定する長さ(mm)
  const double kick_decision = 550;
  const bool kick_flag_tf    = std::get<0>(kick_type_) != model::command::kick_type_t::none;

  if ((ball_vel.norm() > kick_decision) || finishflag_) {
    // executeが呼ばれる間の時間でボールが一定以上移動をしていたら蹴ったと判定
    command.set_velocity({0, 0, 0});
    finishflag_ = true;
    return command;
  }

  switch (state_) {
    case running_state::move: { //ボールの近くまで寄る

      if ((ball_pos - robot_pos).norm() < dist) {
        state_ = running_state::round;
      }
      if (std::abs(util::math::wrap_to_pi(ball_target - robot_ball)) < margin_) {
        state_ = running_state::kick;
      }
      //ボールのそばによる処理
      command.set_position({std::cos(ball_target + pi<double>()) * dist + ball_pos.x(),
                            std::sin(ball_target + pi<double>()) * dist + ball_pos.y(),
                            ball_target});
    } break;

    case running_state::round: { //回り込

      if ((ball_pos - robot_pos).norm() > 1.5 * dist) {
        state_ = running_state::move;
      }
      if (std::abs(util::math::wrap_to_pi(ball_target - robot_ball)) < margin_) {
        state_ = running_state::kick;
      }
      //回りこみ処理
      const double velo =
          (util::math::wrap_to_pi(ball_target + pi<double>() - ball_robot) / pi<double>()) *
          1500;
      const double si = -std::sin(ball_robot) * velo;
      const double co = std::cos(ball_robot) * velo;
      command.set_velocity(
          {si, co, 4.0 * util::math::wrap_to_pi(ball_target - robot_me.theta())});
    } break;

    case running_state::kick: { //キックの状態
      if (std::abs(util::math::wrap_to_pi(ball_target - robot_ball)) > 1.5 * margin_) {
        state_ = running_state::round;
      }
      if ((ball_pos - robot_pos).norm() > 1.5 * dist) {
        state_ = running_state::move;
      }
      //キック処理
      command.set_kick_flag(kick_type_);
      command.set_dribble(dribble_);
      const double velo =
          (util::math::wrap_to_pi(ball_target + pi<double>() - ball_robot) / pi<double>()) *
          1500;
      const double si = -std::sin(ball_robot) * velo;
      const double co = std::cos(ball_robot) * velo;
      const double move_vel =
          !kick_flag_tf ? 0 : 3.0 * ((robot_pos - ball_pos).norm() - 75.0); // 90はドリブラー分
      const Eigen::Vector2d vel = move_vel * (ball_pos - robot_pos).normalized();
      command.set_velocity({vel.x() + si, vel.y() + co,
                            4.0 * util::math::wrap_to_pi(ball_target - robot_me.theta())});
    } break;
  }
  return command;
}
bool kick::finished() const {
  return finishflag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
