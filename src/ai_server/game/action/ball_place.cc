#include <algorithm>
#include <cmath>

#include <boost/math/constants/constants.hpp>

#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

#include "ball_place.h"

using boost::math::constants::half_pi;
using boost::math::constants::pi;
using namespace std::chrono_literals;

namespace ai_server::game::action {

ball_place::ball_place(context& ctx, unsigned int id, const Eigen::Vector2d& target)
    : base(ctx, id),
      state_(running_state::move),
      finished_(false),
      wait_flag_(true),
      target_(target),
      abp_target_(target),
      kick_type_({model::command::kick_type_t::none, 0}) {}

ball_place::running_state ball_place::state() const {
  return state_;
}

void ball_place::set_kick_type(const model::command::kick_flag_t& kick_type) {
  kick_type_ = kick_type;
}

model::command ball_place::execute() {
  model::command command{};
  const auto our_robots           = model::our_robots(world(), team_color());
  const auto& robot               = our_robots.at(id_);
  const auto ball                 = world().ball();
  const Eigen::Vector2d robot_pos = util::math::position(robot);
  const Eigen::Vector2d face_pos =
      robot_pos + Eigen::Rotation2Dd(robot.theta()) * Eigen::Vector2d::UnitX() * 100.0;
  const Eigen::Vector2d ball_pos = util::math::position(ball);
  const Eigen::Vector2d ball_vel = util::math::velocity(ball);
  // 許容誤差(ルール上は10[cm]以内?)
  constexpr double xy_allow = 50.0;
  // dribble
  constexpr int dribble_value = 5;
  // ロボットとボールの距離
  const double dist_b_to_r = (ball_pos - robot_pos).norm();
  // ボールと目標の距離
  const double dist_b_to_t = (ball_pos - target_).norm();
  // ボールと最終目標の距離
  const double dist_b_to_at = (ball_pos - abp_target_).norm();
  // ロボットと目標の距離
  const double dist_r_to_t = (robot_pos - target_).norm();
  // ボールが見えているか?
  const bool ball_visible        = !ball.is_lost();
  const auto& predicted_ball_pos = ball_visible ? ball_pos : face_pos;
  // b基準のaの符号
  auto sign = [](double a, double b) { return ((a > b) - (a < b)); };

  // ボールがフィールド外にあれば目標・モード変更
  {
    if ((std::abs(predicted_ball_pos.x()) > world().field().x_max() - 100.0 &&
         std::abs(predicted_ball_pos.y()) > world().field().y_max() - 100.0 &&
         mode_ == place_mode::pull) ||
        (std::abs(predicted_ball_pos.x()) > world().field().x_max() &&
         std::abs(predicted_ball_pos.y()) > world().field().y_max() &&
         mode_ != place_mode::pull)) {
      target_.x() = (world().field().x_max() - 500.0) * sign(predicted_ball_pos.x(), 0.0);
      target_.y() = (world().field().y_max() - 500.0) * sign(predicted_ball_pos.y(), 0.0);
      mode_       = place_mode::pull;
    } else if ((std::abs(predicted_ball_pos.x()) > world().field().x_max() - 100.0 &&
                mode_ == place_mode::pull) ||
               (std::abs(predicted_ball_pos.x()) > world().field().x_max() &&
                mode_ != place_mode::pull)) {
      target_.x() = (world().field().x_max() - 500.0) * sign(predicted_ball_pos.x(), 0.0);
      target_.y() = predicted_ball_pos.y();
      mode_       = place_mode::pull;
    } else if ((std::abs(predicted_ball_pos.y()) > world().field().y_max() - 100.0 &&
                mode_ == place_mode::pull) ||
               (std::abs(predicted_ball_pos.y()) > world().field().y_max() &&
                mode_ != place_mode::pull)) {
      target_.x() = predicted_ball_pos.x();
      target_.y() = (world().field().y_max() - 500.0) * sign(predicted_ball_pos.y(), 0.0);
      mode_       = place_mode::pull;
    } else {
      target_ = abp_target_;
      mode_   = std::abs(predicted_ball_pos.x()) < world().field().x_max() - 100.0 &&
                      std::abs(predicted_ball_pos.y()) < world().field().y_max() - 100.0
                  ? place_mode::push
                  : place_mode::pull;
    }
    if (mode_ == place_mode::push &&
        std::get<0>(kick_type_) != model::command::kick_type_t::none)
      mode_ = place_mode::pass;
  }

  // 目標角度
  const double theta =
      mode_ == place_mode::push || mode_ == place_mode::pass
          ? std::atan2(target_.y() - robot_pos.y(), target_.x() - robot_pos.x())
          : std::atan2(predicted_ball_pos.y() - target_.y(),
                       predicted_ball_pos.x() - target_.x());

  finished_ = dist_b_to_at < 1.5 * xy_allow && dist_b_to_r > 650.0;
  if (finished_) state_ = running_state::finished;

  switch (state_) {
    // 終了
    case running_state::finished: {
      if (dist_b_to_at > 2.0 * xy_allow || dist_b_to_r < 600.0) {
        state_ = running_state::move;
      }
      finished_ = true;
      command.set_dribble(0);
      command.set_velocity(0.0, 0.0);
      command.set_angle(theta);
    } break;

    // ボールから離れる
    case running_state::leave: {
      const auto now = std::chrono::steady_clock::now();
      if (wait_flag_) {
        begin_     = now;
        wait_flag_ = false;
      }
      if (dist_b_to_r > 650.0) {
        state_ = running_state::finished;
      }
      if (dist_r_to_t > 300.0 &&
          std::abs(std::atan2(0.0 - target_.y(), 0.0 - target_.x()) -
                   std::atan2(robot.y() - target_.y(), robot.x() - target_.x())) >
              pi<double>() / 4.0) {
        // 回り込み
        const Eigen::Vector2d vel =
            1000.0 *
            (Eigen::Rotation2Dd(half_pi<double>()) * (robot_pos - target_).normalized()) *
            sign(std::atan2(robot_pos.y(), robot_pos.x()),
                 std::atan2(ball_pos.y(), ball_pos.x()));
        command.set_velocity(vel);
        command.set_angle(theta);
      } else {
        const Eigen::Vector2d vel = 2000.0 * (robot_pos - target_).normalized();
        command.set_velocity(vel, 0.0);
      }
    } break;

    // 待機
    case running_state::wait: {
      const auto now = std::chrono::steady_clock::now();
      command.set_velocity(0.0, 0.0, 0.0);
      command.set_dribble(0);
      if (wait_flag_) {
        begin_     = now;
        wait_flag_ = false;
      }
      if (now - begin_ >= 1s) {
        state_     = running_state::leave;
        wait_flag_ = true;
      }
    } break;

    // 配置
    case running_state::place: {
      if (dist_b_to_t < xy_allow || (face_pos - target_).norm() < 30.0) {
        // 許容誤差以内にボールがあれば配置完了(ボールが見えていなければロボットの位置で判定)
        state_     = running_state::wait;
        wait_flag_ = true;
      }
      if (ball_visible && ball_vel.norm() < 300.0 &&
          (robot_pos - first_ball_pos_).norm() > 150.0) {
        // ボール速度がある程度落ち、ボール初期位置よりある程度動いていればボール前まで移動する処理に移行
        state_ = running_state::move;
      }
      const double pre_vel =
          mode_ == place_mode::pull || mode_ == place_mode::pass ||
                  std::abs(util::math::wrap_to_pi(theta - robot.theta())) > 0.2
              ? 300
              : 1000;
      const double coef         = std::clamp(3.0 * (face_pos - target_).norm(), 50.0, pre_vel);
      const Eigen::Vector2d vel = coef * (target_ - face_pos).normalized();
      {
        const Eigen::Vector2d tmp(robot_pos + Eigen::Rotation2Dd(robot.theta()) *
                                                  Eigen::Vector2d::UnitX() *
                                                  (abp_target_ - robot_pos).norm());
        if ((tmp - target_).norm() < 50.0) {
          command.set_kick_flag(kick_type_);
        } else {
          command.set_kick_flag(
              model::command::kick_flag_t{model::command::kick_type_t::none, 0});
        }
      }

      // pull -> pushでボールを離さないために角速度を制限
      constexpr double max_omega = pi<double>();
      const double omega = std::clamp(4.0 * util::math::wrap_to_pi(theta - robot.theta()),
                                      -max_omega, max_omega);
      command.set_dribble(dribble_value);
      command.set_velocity(vel, omega);
      first_ball_pos_ = ball_pos;
    } break;

    // ボールを持つ
    case running_state::hold: {
      // ロボットがボールに触れた状態を判断するため、face_pos ではなく robot_pos を使用
      if ((first_ball_pos_ - robot_pos).norm() < 90.0) {
        // ロボットがボールの初期位置に来たら配置に移行する
        state_ = running_state::place;
      }
      // first_ball_pos_ で判定できない場合のために時間でも判定
      const auto now = std::chrono::steady_clock::now();
      if ((ball_pos - robot_pos).norm() > 100.0) begin_ = now;
      if (now - begin_ >= 1s) state_ = running_state::place;

      const double coef         = std::clamp(3.0 * (face_pos - ball_pos).norm(), 50.0, 1000.0);
      const Eigen::Vector2d vel = coef * (first_ball_pos_ - robot_pos).normalized();
      command.set_dribble(dribble_value);
      command.set_velocity(vel);
      command.set_angle(theta);
    } break;

    // 移動
    default: {
      if (std::abs(util::math::wrap_to_pi(
              theta - std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x()))) <
              pi<double>() / 20.0 &&
          dist_b_to_r < 400.0) {
        state_          = running_state::hold;
        first_ball_pos_ = ball_pos;
      }

      if (dist_b_to_r < 500.0) {
        // 回り込み
        // ボールを中心に回転する速度
        const double round_coef = std::clamp(
            1500.0 *
                util::math::wrap_to_pi(
                    (mode_ == place_mode::pull ? pi<double>() : 0.0) +
                    std::atan2(target_.y() - robot_pos.y(), target_.x() - robot_pos.x()) -
                    std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x())) /
                pi<double>(),
            -1000.0, 1000.0);
        const Eigen::Vector2d round_vel = round_coef * (Eigen::Rotation2Dd(half_pi<double>()) *
                                                        (robot_pos - ball_pos).normalized());
        // ボールへ向かう速度
        const double to_ball_coef         = std::min(1000.0, 3.5 * (dist_b_to_r - 100.0));
        const Eigen::Vector2d to_ball_vel = to_ball_coef * (ball_pos - robot_pos).normalized();
        command.set_velocity(ball_vel + round_vel + to_ball_vel);
        command.set_angle(theta);
      } else {
        const Eigen::Vector2d pos(
            ball_pos + Eigen::Rotation2Dd(
                           (mode_ == place_mode::pull ? pi<double>() : 0.0) +
                           std::atan2(ball_pos.y() - target_.y(), ball_pos.x() - target_.x())) *
                           Eigen::Vector2d::UnitX() * 300.0);
        command.set_position(pos, theta);
      }
    }
  }
  return command;
}

bool ball_place::finished() const {
  return finished_;
}

} // namespace ai_server::game::action
