#include "ball.h"
#include <cmath>

namespace ai_server {
namespace filter {
namespace state_observer {
constexpr double ball::fric_coef_;
constexpr double ball::ball_weight_;
constexpr double ball::ball_radius_;
constexpr double ball::air_viscosity_;
constexpr double ball::air_registance_;
constexpr double ball::friction_;
constexpr double ball::quant_limit_x_;
constexpr double ball::quant_limit_y_;
constexpr double ball::lambda_observer_;

ball::ball(const model::ball& ball, std::chrono::system_clock::time_point time)
    : ball_(ball), prev_time_(time) {
  x_hat_[0] = {std::floor(ball.x() / quant_limit_x_) * quant_limit_x_, 0};
  x_hat_[1] = {std::floor(ball.y() / quant_limit_y_) * quant_limit_y_, 0};
  ball_.set_vx(0);
  ball_.set_vy(0);
}

std::optional<model::ball> ball::update(std::optional<model::ball> ball,
                                        std::chrono::system_clock::time_point time) {
  // 対象がロストしたらロストさせる
  // TODO: 任意フレーム補間させる...？
  if (!ball.has_value()) {
    // return std::nullopt;
    ball = ball_;
  }

  Eigen::Matrix<double, 2, 2> A;
  static const Eigen::Matrix<double, 1, 2> C = {1, 0};
  Eigen::Matrix<double, 2, 1> h;

  // 前回呼び出しからの経過時刻[s]
  const auto passed_time = std::chrono::duration<double>(time - prev_time_).count();
  prev_time_             = time;

  auto h1 = [](double fric) { return -2 * lambda_observer_ - (fric + air_registance_); };

  auto h2 = [h1](double fric) {
    return std::pow(lambda_observer_, 2) - h1(fric) * (fric + air_registance_);
  };

  const auto theta = std::atan2(ball_.vy(), ball_.vx());

  // x軸方向について状態推定
  const auto cos_theta = std::cos(theta);

  // μmg < μvの場合、摩擦力fは飽和してμmgに制限される
  //  -> オブザーバゲイン(正確には摩擦係数)を速度に応じて調整することで対応
  //     μmg < μv ... (μmg / v)を新たにμ'とすればμ'vはμmgに制限される
  if (std::abs(friction_ * cos_theta) < std::abs(fric_coef_ * ball_.vx())) {
    // 摩擦が飽和している領域。速度に応じて摩擦係数を調整することで対処
    const auto fric_coef_tuned = std::abs(friction_ * cos_theta / ball_.vx());

    A << 0, 1,                                   // a11  a12
        0, -(fric_coef_tuned + air_registance_); // a21  a22
    h = {h1(fric_coef_tuned), h2(fric_coef_tuned)};
  } else {
    // 摩擦が線形の領域。
    A << 0, 1,                              // a11  a12
        0, -(fric_coef_ + air_registance_); // a21  a22
    h = {h1(fric_coef_), h2(fric_coef_)};
  }

  // 状態観測器の状態方程式は一般に
  // x_hat_dot = (A - hC) * x_hat_ + h * y
  Eigen::Matrix<double, 2, 1> x_hat_dot;
  x_hat_dot =
      (A - h * C) * x_hat_[0] + h * std::floor(ball->x() / quant_limit_x_) * quant_limit_x_;
  x_hat_dot *= passed_time;
  x_hat_[0] += x_hat_dot;
  ball_.set_x(x_hat_[0](0));
  ball_.set_vx(x_hat_[0](1));

  // y軸方向についても同様に状態推定
  const auto sin_theta = std::sin(theta);

  if (std::abs(friction_ * sin_theta) < std::abs(fric_coef_ * ball_.vy())) {
    // 摩擦力が飽和している領域
    const auto fric_coef_tuned = std::abs(friction_ * sin_theta / ball_.vy());

    A << 0, 1,                                   // a11  a12
        0, -(fric_coef_tuned + air_registance_); // a21  a22
    h = {h1(fric_coef_tuned), h2(fric_coef_tuned)};
  } else {
    // 摩擦が線形の領域。
    A << 0, 1,                              // a11  a12
        0, -(fric_coef_ + air_registance_); // a21  a22
    h = {h1(fric_coef_), h2(fric_coef_)};
  }

  x_hat_dot =
      (A - h * C) * x_hat_[1] + h * std::floor(ball->y() / quant_limit_y_) * quant_limit_y_;
  x_hat_dot *= passed_time;
  x_hat_[1] += x_hat_dot;
  ball_.set_y(x_hat_[1](0));
  ball_.set_vy(x_hat_[1](1));

  return ball_;
}
} // namespace state_observer
} // namespace filter
} // namespace ai_server
