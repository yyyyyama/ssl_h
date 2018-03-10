#include <cmath>
#include <boost/math/constants/constants.hpp>
#include "ai_server/util/math/angle.h"
#include "autonomous_ball_place.h"
#include <iostream>
using boost::math::constants::pi;
using namespace std::chrono_literals;

namespace ai_server {
namespace game {
namespace action {

autonomous_ball_place::autonomous_ball_place(const model::world& world, bool is_yellow,
                                             unsigned int id, double target_x, double target_y)
    : base(world, is_yellow, id),
      command_(id),
      state_(running_state::move),
      finished_(false),
      wait_flag_(true),
      round_flag_(false),
      target_x_(target_x),
      target_y_(target_y) {}

autonomous_ball_place::running_state autonomous_ball_place::state() const {
  return state_;
}

model::command autonomous_ball_place::execute() {
  model::command command_{id_};
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto& robot     = our_robots.at(id_);
  const auto ball       = world_.ball();
  // 許容誤差(ルール上は10[cm]以内?)
  const double xy_allow = 50.0;
  // 目標角度
  auto theta = util::math::wrap_to_2pi(std::atan2(target_y_ - ball.y(), target_x_ - ball.x())) +
               pi<double>();
  // ロボットがボールを噛んでいるときのロボットのx座標とボールのx座標の差
  const double distx = std::abs(95.0 * std::cos(theta));
  // ロボットとボールの距離
  const double distance = std::hypot(robot.x() - ball.x(), robot.y() - ball.y());
  // ボールと目標を通る直線
  auto ball_to_target_f = [=](double x) {
    return ((target_y_ - ball.y()) / (target_x_ - ball.x())) * (x - ball.x()) + ball.y();
  };
  // b基準のaの符号
  auto sign = [](double a, double b) { return ((a > b) - (a < b)); };

  if ((std::abs(ball.x() - target_x_) < xy_allow && std::abs(ball.y() - target_y_) < xy_allow &&
       distance > 590.0) ||
      state_ == running_state::finished) {
    // 全条件を満たせば停止
    finished_ = true;
    state_    = running_state::finished;
    command_.set_dribble(0);
    command_.set_velocity({0.0, 0.0, 0.0});
  } else if (state_ == running_state::leave) {
    // ボールから離れる
    finished_      = false;
    const double x = ball.x() + 600.0 * sign(robot.x(), ball.x());
    const double y = (std::abs(robot.x() - ball.x()) <= std::numeric_limits<double>::epsilon())
                         ? ball.y() + 600.0
                         : ((robot.y() - ball.y()) / (robot.x() - ball.x())) * (x - ball.x()) +
                               ball.y(); // 0除算防止
    theta = util::math::wrap_to_2pi(std::atan2(robot.y() - ball.y(), robot.x() - ball.x())) +
            pi<double>();
    command_.set_position({x, y, theta});
  } else if (state_ == running_state::wait) {
    // 待機
    finished_ = false;
    now_      = util::clock_type::now();
    command_.set_dribble(0);
    if (wait_flag_) {
      begin_     = util::clock_type::now();
      wait_flag_ = false;
    }
    if (now_ - begin_ >= 1s) {
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
      // ボールがロボットと離れたらボール前まで移動する処理に移行
      state_ = running_state::move;
    }
    if (std::abs(target_x_ - ball.x()) <= std::numeric_limits<double>::epsilon()) {
      const double x = target_x_;
      const double y = target_y_ + 95.0 * sign(target_y_, ball.y());
      command_.set_dribble(9);
      command_.set_position({x, y, theta});
    } else {
      const double x  = target_x_ + distx * sign(target_x_, ball.x());
      const double y  = ball_to_target_f(x);
      const double vx = 500 * (x - robot.x()) / std::hypot(x, robot.x());
      const double vy = 500 * (y - robot.y()) / std::hypot(y, robot.y());
      command_.set_dribble(9);
      command_.set_velocity({vx, vy, 0.0});
    }
  } else if (state_ == running_state::hold) {
    // ボールを持つ
    finished_ = false;
    std::cout << distance << std::endl;
    if (std::pow(ball.x() - first_ballx_, 2) + std::pow(ball.y() - first_bally_, 2) > 500.0 &&
        distance < 120.0) {
      // ボールが初期位置からある程度離れていてロボットとボールが近ければ配置する処理に移行
      state_ = running_state::place;
    }
    command_.set_dribble(9);
    command_.set_position({ball.x(), ball.y(), theta});
  } else if ((ball_to_target_f(robot.x()) + 5.0 > robot.y() &&
              robot.y() > ball_to_target_f(robot.x()) - 5.0) &&
             ((ball.x() - (distx + 50.0) > robot.x() && robot.x() > target_x_) ||
              (target_x_ > robot.x() && robot.x() > ball.x() - (distx + 50.0)))) {
    finished_    = false;
    state_       = running_state::hold;
    round_flag_  = false;
    first_ballx_ = ball.x();
    first_bally_ = ball.y();
    command_.set_velocity({0.0, 0.0, 0.0});
  } else {
    // 移動
    finished_ = false;
    state_    = running_state::move;
    if ((distance < 200.0 || round_flag_) && distance < 300) {
      // 回り込み
      round_flag_ = true;
      const double vx =
          500 *
          std::sin(
              util::math::wrap_to_2pi(std::atan2(robot.y() - ball.y(), robot.x() - ball.x()))) *
          sign(robot.y(), ball_to_target_f(robot.x())) * sign(target_x_, ball.x());
      const double vy =
          -500 *
          std::cos(
              util::math::wrap_to_2pi(std::atan2(robot.y() - ball.y(), robot.x() - ball.x()))) *
          sign(robot.y(), ball_to_target_f(robot.x())) * sign(target_x_, ball.x());
      command_.set_velocity({vx, vy, 0.0});
    } else {
      const double x =
          ball.x() + (distx + 50.0) * ((ball.x() > robot.x()) - (ball.x() < robot.x()));
      const double y =
          (std::abs(target_x_ - ball.x()) <= std::numeric_limits<double>::epsilon())
              ? ball.y()
              : ball_to_target_f(x);
      command_.set_position({x, y, theta});
    }
  }
  return command_;
}

bool autonomous_ball_place::finished() const {
  return finished_;
};
} // namespace action
} // namespace game
} // namespace ai_server
