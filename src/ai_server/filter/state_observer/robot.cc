#include "robot.h"
#include <cmath>

#include "ai_server/util/math/angle.h"

namespace ai_server {
namespace filter {
namespace state_observer {
constexpr double robot::lambda_robot_observer_;
constexpr double robot::decay_vel_;
constexpr double robot::decay_acc_;

robot::robot(std::recursive_mutex& mutex, robot::writer_func_type wf,
             std::chrono::system_clock::duration time)
    : base(mutex, wf),
      prev_time_(std::chrono::system_clock::time_point::min()),
      capture_time_(std::chrono::system_clock::time_point::min()),
      receive_time_(std::chrono::system_clock::time_point::min()),
      lost_duration_(time) {
  x_hat_.fill(decltype(x_hat_)::value_type::Zero());
}

void robot::observe(double vx, double vy) {
  std::unique_lock lock{mutex()};

  const auto A = (Eigen::Matrix<double, 3, 3>{} << 0, 1, 0, // a11, a12, a13
                  0, 0, 1,                                  // a21, a22, a23
                  0, -decay_vel_, -decay_acc_               // a31, a32, a33
                  )
                     .finished();
  const auto B = (Eigen::Matrix<double, 3, 1>{} << 0, 0, 1).finished();
  const auto C = (Eigen::Matrix<double, 1, 3>{} << 1, 0, 0).finished();
  // オブザーバゲインh1
  constexpr auto h1 = 3 * lambda_robot_observer_ - decay_acc_;
  // オブザーバゲインh2
  constexpr auto h2 =
      3 * lambda_robot_observer_ * lambda_robot_observer_ - decay_acc_ * h1 - decay_vel_;
  // オブザーバゲイン[h1, h2, h3]T
  const auto h = (Eigen::Matrix<double, 3, 1>{} << h1, h2,
                  std::pow(lambda_robot_observer_, 3) - decay_vel_ * h1 - decay_acc_ * h2)
                     .finished();

  // 観測した時間からlost_duration_経過していたらロストさせる
  if (std::chrono::system_clock::now() - capture_time_ > lost_duration_) {
    capture_time_ = std::chrono::system_clock::time_point::min();
    write(std::nullopt);
    return;
  }

  auto robot = raw_value_.value_or(prev_state_);

  // 前回値読み込み時からの経過時刻[s]
  const auto passed_time = std::chrono::duration<double>(receive_time_ - prev_time_).count();
  // 非常に短い間隔で呼び出されたら直前の値を返す
  if (std::abs(passed_time) < std::numeric_limits<double>::epsilon()) {
    write(prev_state_);
    return;
  }
  prev_time_ = receive_time_;

  // オイラー法を用いて状態更新を行う
  // 状態方程式：d(x_hat)/dt = (A * x_hat) + (B * u) + h * (y - y_hat)
  Eigen::Matrix<double, 3, 1> x_hat_dot;
  // x軸方向
  // 時間更新
  x_hat_[0] += (A * x_hat_[0] + B * vx) * passed_time;
  // 観測情報に基づき補正
  const auto error_x = C * x_hat_[0] - robot.x();
  x_hat_dot          = -h * error_x;
  x_hat_dot *= passed_time;
  x_hat_[0] += x_hat_dot;
  robot.set_x(x_hat_[0](0));
  robot.set_vx(x_hat_[0](1));
  robot.set_ax(x_hat_[0](2));

  // y軸方向
  // 時間更新
  x_hat_[1] += (A * x_hat_[1] + B * vy) * passed_time;
  // 観測情報に基づき補正
  const auto error_y = C * x_hat_[1] - robot.y();
  x_hat_dot          = -h * error_y;
  x_hat_dot *= passed_time;
  x_hat_[1] += x_hat_dot;
  robot.set_y(x_hat_[1](0));
  robot.set_vy(x_hat_[1](1));
  robot.set_ay(x_hat_[1](2));

  // 角速度の計算
  robot.set_omega(util::math::wrap_to_pi(robot.theta() - prev_state_.theta()) / passed_time);
  robot.set_alpha((robot.omega() - prev_state_.omega()) / passed_time);
  prev_state_ = robot;

  write(robot);
}

void robot::set_raw_value(std::optional<model::robot> value,
                          std::chrono::system_clock::time_point time) {
  std::unique_lock lock{mutex()};

  if (value.has_value()) {
    if (capture_time_ == std::chrono::system_clock::time_point::min()) {
      model::robot r = *value;
      x_hat_[0]      = {r.x(), 0, 0};
      x_hat_[1]      = {r.y(), 0, 0};
      write(r);
    }
    capture_time_ = time;
  }
  raw_value_    = value;
  receive_time_ = time;
}
} // namespace state_observer
} // namespace filter
} // namespace ai_server
