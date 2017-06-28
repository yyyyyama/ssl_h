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

ball::ball(const model::ball& ball, std::chrono::high_resolution_clock::time_point time)
    : ball_(ball), prev_time_(time) {
  x_hat_[0] << (int)(ball.x() / quant_limit_x_) * quant_limit_x_, 0;
  x_hat_[1] << (int)(ball.y() / quant_limit_y_) * quant_limit_y_, 0;
  ball_.set_vx(0);
  ball_.set_vy(0);
}

model::ball ball::update(const model::ball& ball,
                         std::chrono::high_resolution_clock::time_point time) {
  Eigen::Matrix<double, 2, 2> A;
  static const Eigen::Matrix<double, 1, 2> C(1, 0);
  Eigen::Matrix<double, 2, 1> h;

  // 前回呼び出しからの経過時刻[s]
  auto passed_time = std::chrono::duration<double>(time - prev_time_).count();
  prev_time_       = time;

  // 状態変数行列から位置を取り出すやつ
  auto to_pos = [](const Eigen::Matrix<double, 2, 1>& x_hat) {
    Eigen::Matrix<double, 1, 2> C;
    C << 1, 0;
    return C * x_hat;
  };

  // 状態変数行列から速度を取り出すやつ
  auto to_vel = [](const Eigen::Matrix<double, 2, 1>& x_hat) {
    Eigen::Matrix<double, 1, 2> C;
    C << 0, 1;
    return C * x_hat;
  };

  auto h1 = [](double fric) { return -2 * lambda_observer_ - (fric + air_registance_); };

  auto h2 = [h1](double fric) {
    return std::pow(lambda_observer_, 2) - h1(fric) * (fric + air_registance_);
  };

  auto theta = std::atan(ball_.vx() / ball_.vy());

  // x軸方向について状態推定
  auto cos_theta = std::cos(theta);

  // μmg < μvの場合、摩擦力fは飽和してμmgに制限される
  //  -> オブザーバゲイン(正確には摩擦係数)を速度に応じて調整することで対応
  //     μmg < μv ... (μmg / v)を新たにμ'とすればμ'vはμmgに制限される
  if (std::abs(friction_ * cos_theta) < std::abs(fric_coef_ * ball_.vx())) {
    // 摩擦が飽和している領域。速度に応じて摩擦係数を調整することで対処
    auto fric_coef_tuned = std::abs(friction_ * cos_theta / ball_.vx());

    A << 0, 1, 0, -(fric_coef_tuned + air_registance_);
    h << h1(fric_coef_tuned), h2(fric_coef_tuned);
  } else {
    // 摩擦が線形の領域。
    A << 0, 1, 0, -(fric_coef_ + air_registance_);
    h << h1(fric_coef_), h2(fric_coef_);
  }

  // 状態観測器の状態方程式は一般に
  // x_hat_dot = (A - hC) * x_hat_ + h * y
  Eigen::Matrix<double, 2, 1> x_hat_dot;
  x_hat_dot = (A - h * C) * x_hat_[0] + h * (int)(ball.x() / quant_limit_x_) * quant_limit_x_;
  x_hat_dot *= passed_time;
  x_hat_[0] += x_hat_dot;
  ball_.set_x(to_pos(x_hat_[0]));
  ball_.set_vx(to_vel(x_hat_[0]));

  // y軸方向についても同様に状態推定
  auto sin_theta = std::sin(theta);

  if (std::abs(friction_ * sin_theta) < std::abs(fric_coef_ * ball_.vy())) {
    // 摩擦力が飽和している領域
    auto fric_coef_tuned = std::abs(friction_ * sin_theta / ball_.vy());

    A << 0, 1, 0, -(fric_coef_tuned + air_registance_);
    h << h1(fric_coef_tuned), h2(fric_coef_tuned);
  } else {
    // 摩擦が線形の領域。
    A << 0, 1, 0, -(fric_coef_ + air_registance_);
    h << h1(fric_coef_), h2(fric_coef_);
  }

  x_hat_dot = (A - h * C) * x_hat_[1] + h * (int)(ball.y() / quant_limit_y_) * quant_limit_y_;
  x_hat_dot *= passed_time;
  x_hat_[1] += x_hat_dot;
  ball_.set_y(to_pos(x_hat_[1]));
  ball_.set_vy(to_vel(x_hat_[1]));

  return ball_;
}
} // namespace state_observer
} // namespace filter
} // namespace ai_server
