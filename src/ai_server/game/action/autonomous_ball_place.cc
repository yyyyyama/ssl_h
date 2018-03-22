#include <cmath>
#include <boost/math/constants/constants.hpp>
#include "ai_server/util/math/angle.h"
#include "autonomous_ball_place.h"

using boost::math::constants::pi;
using namespace std::chrono_literals;

namespace ai_server {
namespace game {
namespace action {

autonomous_ball_place::autonomous_ball_place(const model::world& world, bool is_yellow,
                                             unsigned int id, Eigen::Vector2d target)
    : base(world, is_yellow, id),
      command_(id),
      state_(running_state::move),
      finished_(false),
      wait_flag_(true),
      round_flag_(false),
      target_(target) {}

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
  auto theta =
      util::math::wrap_to_2pi(std::atan2(ball.y() - target_.y(), ball.x() - target_.x()));
  // ロボットがボールを噛んでいるときのロボットのx座標とボールのx座標の差
  const double distx = std::abs(95.0 * std::cos(theta));
  // ロボットとボールの距離
  const double dist_b_to_r = std::hypot(robot.x() - ball.x(), robot.y() - ball.y());
  // ボールと目標の距離
  const double dist_b_to_t = std::hypot(ball.x() - target_.x(), ball.y() - target_.y());
  // ロボットと目標の距離
  const double dist_r_to_t = std::hypot(robot.x() - target_.x(), robot.y() - target_.y());
  // ボールと目標を通る直線
  auto ball_to_target_f = [&](double x) {
    return ((target_.y() - ball.y()) / (target_.x() - ball.x())) * (x - ball.x()) + ball.y();
  };
  // b基準のaの符号
  auto sign = [](double a, double b) { return ((a > b) - (a < b)); };
  // ボールが見えているか?
  auto ball_visible = [&]() {
    return std::hypot(ball.x() - first_ballx_, ball.y() - first_bally_) >
           std::numeric_limits<double>::epsilon();
  };

  if ((dist_b_to_t < xy_allow &&
       std::hypot(robot.x() - target_.x(), robot.y() - target_.y()) > 590.0) ||
      state_ == running_state::finished) {
    // 全条件を満たせば停止
    finished_ = true;
    state_    = running_state::finished;
    command_.set_dribble(0);
    command_.set_velocity({0.0, 0.0, 0.0});
  } else if (state_ == running_state::leave) {
    // ボールから離れる
    finished_ = false;
    if (dist_b_to_t > 2 * xy_allow && ball_visible()) {
      // ボールが最終目標からいくらか離れたら最初から
      state_ = running_state::move;
    }
    if (dist_r_to_t > 150.0 &&
        std::abs(util::math::wrap_to_2pi(std::atan2(0.0 - target_.y(), 0.0 - target_.x())) -
                 util::math::wrap_to_2pi(std::atan2(
                     robot.y() - target_.y(), robot.x() - target_.x()))) > pi<double>() / 4.0) {
      // 回り込み
      const double vx = 1000 *
                        std::sin(util::math::wrap_to_2pi(
                            std::atan2(robot.y() - target_.y(), robot.x() - target_.x()))) *
                        sign(util::math::wrap_to_2pi(std::atan2(ball.y(), ball.x())),
                             util::math::wrap_to_2pi(std::atan2(robot.y(), robot.x())));
      const double vy = -1000 *
                        std::cos(util::math::wrap_to_2pi(
                            std::atan2(robot.y() - target_.y(), robot.x() - target_.x()))) *
                        sign(util::math::wrap_to_2pi(std::atan2(ball.y(), ball.x())),
                             util::math::wrap_to_2pi(std::atan2(robot.y(), robot.x())));
      command_.set_velocity({vx, vy, 0.0});
    } else {
      const double vx = 1000.0 * (robot.x() - target_.x()) / dist_r_to_t;
      const double vy = 1000.0 * (robot.y() - target_.y()) / dist_r_to_t;
      command_.set_velocity({vx, vy, 0.0});
    }
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
  } else if ((dist_b_to_t < xy_allow) ||
             (state_ == running_state::place && std::abs(target_.x()) > std::abs(robot.x()) &&
              std::abs(target_.y()) > std::abs(robot.y()) && !ball_visible() &&
              std::abs(dist_r_to_t - 120.0) < 10.0)) {
    // 許容誤差以内にボールがあれば配置完了(ボールが見えていなければロボットの位置で判定)
    finished_  = false;
    state_     = running_state::wait;
    wait_flag_ = true;
    command_.set_dribble(0);
    command_.set_velocity({0.0, 0.0, 0.0});
  } else if (state_ == running_state::place) {
    // 配置
    finished_ = false;
    state_    = running_state::place;
    if (ball_visible() && std::hypot(ball.vx(), ball.vy()) < 10.0 &&
        std::hypot(robot.x() - first_ballx_, robot.y() - first_bally_) > 300.0) {
      // ボール速度がある程度落ち、ボール初期位置よりある程度動いていればボール前まで移動する処理に移行(ボールが全く移動していないとき、ボールはカメラから見えていないと考える)
      state_ = running_state::move;
    }
    if (std::abs(target_.x() - ball.x()) <= std::numeric_limits<double>::epsilon()) {
      const double x = target_.x();
      const double y = target_.y() + 95.0 * sign(target_.y(), ball.y());
      command_.set_dribble(9);
      command_.set_position({x, y, theta});
    } else {
      const double x  = target_.x() + distx * sign(target_.x(), ball.x());
      const double y  = ball_to_target_f(x);
      const double vx = 500.0 * (x - robot.x()) / std::hypot(x - robot.x(), y - robot.y());
      const double vy = 500.0 * (y - robot.y()) / std::hypot(x - robot.x(), y - robot.y());
      command_.set_dribble(9);
      command_.set_velocity({vx, vy, 0.0});
    }
    first_ballx_ = ball.x();
    first_bally_ = ball.y();
  } else if (state_ == running_state::hold) {
    // ボールを持つ
    finished_ = false;
    if (std::hypot(robot.x() - first_ballx_, robot.y() - first_bally_) < 85.0) {
      // ロボットがボールの初期位置に来たら配置に移行する
      state_ = running_state::place;
      command_.set_velocity({0.0, 0.0, 0.0});
      return command_;
    }
    command_.set_dribble(9);
    command_.set_position({first_ballx_, first_bally_, theta});
  } else if (std::abs(theta - util::math::wrap_to_2pi(
                                  std::atan2(ball.y() - robot.y(), ball.x() - robot.x()))) <
                 pi<double>() / 18.0 &&
             dist_b_to_r < 400.0) {
    finished_    = false;
    state_       = running_state::hold;
    round_flag_  = false;
    first_ballx_ = ball.x();
    first_bally_ = ball.y();
    command_.set_dribble(9);
    command_.set_velocity({0.0, 0.0, 0.0});
  } else {
    // 移動
    finished_ = false;
    state_    = running_state::move;
    if ((dist_b_to_r < 200.0 || round_flag_) && dist_b_to_r < 400.0) {
      // 回り込み
      round_flag_ = true;
      const double vx =
          700 *
          std::sin(
              util::math::wrap_to_2pi(std::atan2(robot.y() - ball.y(), robot.x() - ball.x()))) *
          sign(robot.y(), ball_to_target_f(robot.x())) * sign(target_.x(), ball.x());
      const double vy =
          -700 *
          std::cos(
              util::math::wrap_to_2pi(std::atan2(robot.y() - ball.y(), robot.x() - ball.x()))) *
          sign(robot.y(), ball_to_target_f(robot.x())) * sign(target_.x(), ball.x());
      command_.set_velocity({vx, vy, 0.0});
    } else {
      const double x = ball.x() + distx * sign(ball.x(), robot.x());
      const double y =
          (std::abs(target_.x() - ball.x()) <= std::numeric_limits<double>::epsilon())
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
