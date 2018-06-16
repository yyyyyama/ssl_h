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
      target_(target),
      abp_target_(target) {}

autonomous_ball_place::running_state autonomous_ball_place::state() const {
  return state_;
}

model::command autonomous_ball_place::execute() {
  model::command command_{id_};
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto& robot     = our_robots.at(id_);
  const auto ball       = world_.ball();

  // ボールがフィールド外にあれば目標・モード変更
  if ((std::abs(ball.x()) > world_.field().x_max() - 100.0 &&
       std::abs(ball.y()) > world_.field().y_max() - 100.0 && mode_ == place_mode::pull) ||
      (std::abs(ball.x()) > world_.field().x_max() &&
       std::abs(ball.y()) > world_.field().y_max() && mode_ != place_mode::pull)) {
    target_.x() = (world_.field().x_max() - 500.0) * ball.x() / std::abs(ball.x());
    target_.y() = (world_.field().y_max() - 500.0) * ball.y() / std::abs(ball.y());
    mode_       = place_mode::pull;
  } else if ((std::abs(ball.x()) > world_.field().x_max() - 100.0 &&
              mode_ == place_mode::pull) ||
             (std::abs(ball.x()) > world_.field().x_max() && mode_ != place_mode::pull)) {
    target_.x() = (world_.field().x_max() - 500.0) * ball.x() / std::abs(ball.x());
    target_.y() = ball.y() + 10.0;
    mode_       = place_mode::pull;
  } else if ((std::abs(ball.y()) > world_.field().y_max() - 100.0 &&
              mode_ == place_mode::pull) ||
             (std::abs(ball.y()) > world_.field().y_max() && mode_ != place_mode::pull)) {
    target_.x() = ball.x() + 10.0;
    target_.y() = (world_.field().y_max() - 500.0) * ball.y() / std::abs(ball.y());
    mode_       = place_mode::pull;
  } else {
    target_ = abp_target_;
    mode_   = std::abs(ball.x()) < world_.field().x_max() - 100.0 &&
                    std::abs(ball.y()) < world_.field().y_max() - 100.0
                ? place_mode::push
                : place_mode::pull;
  }

  // 許容誤差(ルール上は10[cm]以内?)
  const double xy_allow            = 50.0;
  const unsigned int dribble_value = 5;
  // 目標角度
  auto theta =
      mode_ == place_mode::push || mode_ == place_mode::pass
          ? util::math::wrap_to_2pi(std::atan2(target_.y() - ball.y(), target_.x() - ball.x()))
          : util::math::wrap_to_2pi(std::atan2(ball.y() - target_.y(), ball.x() - target_.x()));
  // ロボットとボールの距離
  const double dist_b_to_r = std::hypot(robot.x() - ball.x(), robot.y() - ball.y());
  // ボールと目標の距離
  const double dist_b_to_t = std::hypot(ball.x() - target_.x(), ball.y() - target_.y());
  // ボールと最終目標の距離
  const double dist_b_to_at =
      std::hypot(ball.x() - abp_target_.x(), ball.y() - abp_target_.y());
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

  if ((dist_b_to_at < xy_allow &&
       std::hypot(robot.x() - abp_target_.x(), robot.y() - abp_target_.y()) > 600.0) ||
      state_ == running_state::finished) {
    // 全条件を満たせば停止
    finished_ = true;
    state_    = running_state::finished;
    command_.set_dribble(0);
    command_.set_velocity({0.0, 0.0, 0.0});
  } else if (state_ == running_state::leave) {
    // ボールから離れる
    finished_ = false;
    now_      = util::clock_type::now();
    if (wait_flag_) {
      begin_     = util::clock_type::now();
      wait_flag_ = false;
    }
    if (dist_b_to_t > 2 * xy_allow && ball_visible() && (now_ - begin_) > 2s &&
        std::hypot(target_.x() - robot.x(), target_.y() - robot.y()) > 200.0) {
      // ボールが目標からいくらか離れたら最初から
      state_ = running_state::move;
    }
    if (std::hypot(robot.x() - abp_target_.x(), robot.y() - abp_target_.y()) > 600.0 &&
        dist_b_to_at <= 2 * xy_allow) {
      state_ = running_state::finished;
    }
    if (std::hypot(robot.x() - target_.x(), robot.y() - target_.y()) > 300.0 &&
        dist_b_to_t <= 2 * xy_allow) {
      state_ = running_state::move;
    }
    if (dist_r_to_t > 300.0 &&
        std::abs(util::math::wrap_to_2pi(std::atan2(0.0 - target_.y(), 0.0 - target_.x())) -
                 util::math::wrap_to_2pi(std::atan2(
                     robot.y() - target_.y(), robot.x() - target_.x()))) > pi<double>() / 4.0) {
      // 回り込み
      const double vx = 1000.0 *
                        std::sin(util::math::wrap_to_2pi(
                            std::atan2(robot.y() - target_.y(), robot.x() - target_.x()))) *
                        sign(util::math::wrap_to_2pi(std::atan2(ball.y(), ball.x())),
                             util::math::wrap_to_2pi(std::atan2(robot.y(), robot.x())));
      const double vy = -1000.0 *
                        std::cos(util::math::wrap_to_2pi(
                            std::atan2(robot.y() - target_.y(), robot.x() - target_.x()))) *
                        sign(util::math::wrap_to_2pi(std::atan2(ball.y(), ball.x())),
                             util::math::wrap_to_2pi(std::atan2(robot.y(), robot.x())));
      command_.set_velocity({vx, vy, 0.0});
    } else {
      const double vx = 2000.0 * (robot.x() - target_.x()) / dist_r_to_t;
      const double vy = 2000.0 * (robot.y() - target_.y()) / dist_r_to_t;
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
      state_     = running_state::leave;
      wait_flag_ = true;
    }
  } else if (dist_b_to_t < xy_allow ||
             (state_ == running_state::place && !(ball_visible()) &&
              std::hypot(robot.x() + 100.0 * std::cos(robot.theta()) - target_.x(),
                         robot.y() + 100.0 * std::sin(robot.theta()) - target_.y()) < 30.0)) {
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
    if (ball_visible() && std::hypot(ball.vx(), ball.vy()) < 300.0 &&
        std::hypot(robot.x() - first_ballx_, robot.y() - first_bally_) > 150.0) {
      // ボール速度がある程度落ち、ボール初期位置よりある程度動いていればボール前まで移動する処理に移行(ボールが全く移動していないとき、ボールはカメラから見えていないと考える)
      state_ = running_state::move;
    }
    const double pre_vel = mode_ == place_mode::pull ? 700 : 1000;
    const double x =
        target_.x() + 110.0 * std::cos(util::math::wrap_to_2pi(robot.theta() - pi<double>()));
    const double y =
        target_.y() + 110.0 * std::sin(util::math::wrap_to_2pi(robot.theta() - pi<double>()));
    const double vel =
        dist_r_to_t > 700.0
            ? pre_vel
            : std::pow(dist_r_to_t * std::pow(pre_vel - 150, 0.5) / 700.0, 2) + 150.0;
    /*
    const double vel =
        dist_r_to_t > 700.0 ? pre_vel : dist_r_to_t * (pre_vel - 150) / 700.0 + 150.0;
    */
    const double vx = vel * (x - robot.x()) / std::hypot(x - robot.x(), y - robot.y());
    const double vy = vel * (y - robot.y()) / std::hypot(x - robot.x(), y - robot.y());
    const double omega =
        (theta - robot.theta()) / 4.0 + pi<double>() / 18.0 * sign(theta, robot.theta());
    command_.set_dribble(dribble_value);
    command_.set_velocity({vx, vy, omega});
    first_ballx_ = ball.x();
    first_bally_ = ball.y();
  } else if (state_ == running_state::hold) {
    // ボールを持つ
    finished_ = false;
    if (std::hypot(robot.x() - first_ballx_, robot.y() - first_bally_) < 90.0) {
      // ロボットがボールの初期位置に来たら配置に移行する
      state_ = running_state::place;
      return command_;
    }
    const double x = first_ballx_;
    const double y = first_bally_;
    const double vel =
        dist_b_to_r > 500.0 ? 600.0 : std::pow(dist_b_to_r * 17.0 / 500.0, 2) + 111.0;
    const double vx = vel * (x - robot.x()) / std::hypot(x - robot.x(), y - robot.y());
    const double vy = vel * (y - robot.y()) / std::hypot(x - robot.x(), y - robot.y());
    /*
    const double omega =
        (theta - robot.theta()) / 2.0 + pi<double>() / 18.0 * sign(theta, robot.theta());
    */
    const double r_to_btheta =
        util::math::wrap_to_2pi(std::atan2(ball.y() - robot.y(), ball.x() - robot.x()));
    const double omega = (r_to_btheta - robot.theta()) / 2.0 +
                         pi<double>() / 18.0 * sign(r_to_btheta, robot.theta());
    command_.set_dribble(dribble_value);
    command_.set_velocity({vx, vy, omega});
  } else if (std::abs(theta - util::math::wrap_to_2pi(
                                  std::atan2(ball.y() - robot.y(), ball.x() - robot.x()))) <
                 pi<double>() / 40.0 &&
             dist_b_to_r < 500.0) {
    finished_    = false;
    state_       = running_state::hold;
    round_flag_  = false;
    first_ballx_ = ball.x();
    first_bally_ = ball.y();
    const double omega =
        (theta - robot.theta()) / 2.0 + pi<double>() / 8.0 * sign(theta, robot.theta());
    command_.set_dribble(dribble_value);
    command_.set_velocity({0.0, 0.0, omega});
  } else {
    // 移動
    finished_ = false;
    state_    = running_state::move;
    const double base_x =
        mode_ == place_mode::pull ? ball.x() : ball.x() + 80.0 * std::cos(theta);
    const double base_y =
        mode_ == place_mode::pull ? ball.y() : ball.y() + 80.0 * std::sin(theta);
    //基準と目標の距離
    const double dist_ba_to_r = std::hypot(base_x - robot.x(), base_y - robot.y());
    if (((120.0 < dist_ba_to_r && dist_ba_to_r < 300.0) || round_flag_) &&
        dist_ba_to_r < 600.0) {
      // 回り込み
      round_flag_ = true;
      const int r_sign =
          mode_ == place_mode::pull
              ? sign(robot.y(), ball_to_target_f(robot.x())) * sign(target_.x(), ball.x())
              : -1 * sign(robot.y(), ball_to_target_f(robot.x())) * sign(target_.x(), ball.x());
      const double vx = 800.0 *
                        std::sin(util::math::wrap_to_2pi(
                            std::atan2(robot.y() - base_y, robot.x() - base_x))) *
                        r_sign;
      const double vy = -800.0 *
                        std::cos(util::math::wrap_to_2pi(
                            std::atan2(robot.y() - base_y, robot.x() - base_x))) *
                        r_sign;
      const double omega =
          (theta - robot.theta()) / 2.0 + pi<double>() / 8.0 * sign(theta, robot.theta());
      command_.set_velocity({vx, vy, omega});
    } else {
      const double x = ball.x() - 150.0 * std::cos(robot.theta());
      const double y = ball.y() - 150.0 * std::sin(robot.theta());
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
