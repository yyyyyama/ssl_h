#include <chrono>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include "ai_server/util/math/angle.h"
#include "ai_server/util/time.h"
#include "autonomous_ball_place.h"

using boost::math::constants::pi;

namespace ai_server {
namespace game {
namespace action {

autonomous_ball_place::autonomous_ball_place(const model::world& world, bool is_yellow,
                                             unsigned int id)
    : base(world, is_yellow, id), command_(id) {}

autonomous_ball_place::running_state autonomous_ball_place::state() const {
  return state_;
}

void autonomous_ball_place::place_to(double target_x, double target_y) {
  target_x_ = target_x;
  target_y_ = target_y;
}

model::command autonomous_ball_place::execute() {
  model::command command_{id_};
  // 許容誤差(ルール上は10[cm]以内?)
  const double xy_allow = 50.0;
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto& robot     = our_robots.at(id_);
  const auto ball       = world_.ball();
  // 目標
  double x_;
  double y_;
  auto theta_ =
      util::math::wrap_to_2pi(std::atan2(target_y_ - ball.y(), target_x_ - ball.x())) +
      pi<double>();
  // ロボットがボールを噛んでいるときのロボットのx座標とボールのx座標の差
  const double distx = std::abs(95.0 * std::cos(theta_));
  // ロボットとボールの距離
  const double distance =
      std::pow(std::pow(robot.x() - ball.x(), 2) + std::pow(robot.y() - ball.y(), 2), 1.0 / 2);
  const std::chrono::seconds wait_time_(1);

  if (std::abs(ball.x() - target_x_) < xy_allow && std::abs(ball.y() - target_y_) < xy_allow &&
      distance > 590.0) {
    // 全条件を満たせば停止
    finished_ = true;
    state_    = running_state::finished;
    command_.set_dribble(0);
    command_.set_velocity({0.0, 0.0, 0.0});
  } else if (state_ == running_state::leave) {
    // ボールから離れる
    finished_ = false;
    x_        = ball.x() + 600.0 * ((robot.x() > ball.x()) - (robot.x() < ball.x()));
    y_        = (robot.x() == ball.x())
             ? ball.y() + 600.0
             : ((robot.y() - ball.y()) / (robot.x() - ball.x())) * (x_ - ball.x()) +
                   ball.y(); // 0除算防止
    theta_ = util::math::wrap_to_2pi(std::atan2(robot.y() - ball.y(), robot.x() - ball.x())) +
             pi<double>();
    command_.set_position({x_, y_, theta_});
  } else if (state_ == running_state::wait) {
    // 待機
    finished_ = false;
    now_      = std::chrono::system_clock::now();
    if (wait_flag_) {
      begin_     = std::chrono::system_clock::now();
      wait_flag_ = false;
    }
    if (std::chrono::duration_cast<std::chrono::seconds>(now_ - begin_) >= wait_time_) {
      state_ = running_state::leave;
    }
  } else if (std::abs(ball.x() - target_x_) < xy_allow &&
             std::abs(ball.y() - target_y_) < xy_allow) {
    // 配置完了
    finished_  = false;
    state_     = running_state::wait;
    wait_flag_ = true;
    command_.set_dribble(0);
    command_.set_velocity({0.0, 0.0, 0.0});
  } else if (state_ == running_state::place) {
    // 配置
    finished_ = false;
    state_    = running_state::place;
    if (distance > 170.0) {
      state_ = running_state::hold;
    }
    if (target_x_ == ball.x()) {
      x_ = target_x_;
      y_ = target_y_ + 95.0 * ((target_y_ > ball.y()) - (target_y_ < ball.y()));
    } else {
      x_ = target_x_ + distx * ((target_x_ > ball.x()) - (target_x_ < ball.x()));
      y_ = ((target_y_ - ball.y()) / (target_x_ - ball.x())) * (x_ - target_x_) + target_y_;
    }
    command_.set_dribble(9);
    command_.set_position({x_, y_, theta_});
  } else if (state_ == running_state::hold) {
    // ボールを持つ
    finished_ = false;
    if (std::pow(ball.x() - first_ballx_, 2) + std::pow(ball.y() - first_bally_, 2) >
        500.0 /*ボールがある程度動いたら持ったとみなす*/) {
      state_ = running_state::place;
    }
    command_.set_dribble(9);
    command_.set_position({ball.x(), ball.y(), theta_});
  } else if (distance < 170.0) {
    finished_    = false;
    state_       = running_state::hold;
    first_ballx_ = ball.x();
    first_bally_ = ball.y();
    command_.set_velocity({0.0, 0.0, 0.0});
  } else {
    // 移動
    state_       = running_state::move;
    first_ballx_ = ball.x();
    first_bally_ = ball.y();
    x_ = ball.x() + (distx + 50.0) * ((ball.x() > robot.x()) - (ball.x() < robot.x()));
    y_ = (robot.x() == ball.x())
             ? ball.y()
             : ((target_y_ - ball.y()) / (target_x_ - ball.x())) * (x_ - ball.x()) + ball.y();
    theta_ = util::math::wrap_to_2pi(std::atan2(target_y_ - ball.y(), target_x_ - ball.x())) +
             pi<double>();
    command_.set_position({x_, y_, theta_});
  }
  return command_;
}

bool autonomous_ball_place::finished() const {
  return finished_;
};
} // namespace action
} // namespace game
} // namespace ai_server
