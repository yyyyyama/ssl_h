#ifndef AI_SERVER_FILTER_STATE_OBSERVER_BALL_H
#define AI_SERVER_FILTER_STATE_OBSERVER_BALL_H

#include "ai_server/filter/base.h"
#include "ai_server/model/ball.h"
#include <Eigen/Core>
#include <array>
#include <boost/math/constants/constants.hpp>

namespace ai_server {
namespace filter {
namespace state_observer {
class ball : public base<model::ball, timing::same> {
private:
  // 係数群
  static constexpr double fric_coef_     = 0.04;         // 床とボールの摩擦係数
  static constexpr double ball_weight_   = 45.93 / 1000; // ボールの重さ[kg]
  static constexpr double ball_radius_   = 42.67 / 2000; // ボールの半径[m]
  static constexpr double air_viscosity_ = 1.822e-5;     // 空気の粘度[Pa・s]
  // ボールの粘性抵抗係数[kg/s]
  static constexpr double air_registance_ =
      6 * boost::math::double_constants::pi * air_viscosity_ * ball_radius_;
  static constexpr double friction_ =
      fric_coef_ * ball_weight_ * 9.8; // 床とボールの最大動摩擦力[N]
  // 暫定(TODO?: カメラの量子化限界(奥行き方向)。フィールド奥行き[mm] / カメラ分解能)
  static constexpr double quant_limit_x_ = 1.0;
  // 暫定(TODO?: カメラの量子化限界(幅方向)。フィールド幅[mm] / カメラ分解能)
  static constexpr double quant_limit_y_   = 1.0;
  static constexpr double lambda_observer_ = -9; // ボールの状態オブザーバの極

  model::ball ball_;
  std::chrono::system_clock::time_point prev_time_;  // 前回呼び出された時刻
  std::array<Eigen::Matrix<double, 2, 1>, 2> x_hat_; // 状態変数行列 [位置, 速度]T

public:
  ball() = delete;

  /// @brief コンストラクタ
  /// @param ball ボールの初期位置
  /// @param time インスタンス生成時の時刻
  explicit ball(const model::ball& ball, std::chrono::system_clock::time_point time);
  ball(const ball&) = default;

  /// @brief	状態オブザーバの状態を更新する
  /// @param ball Visionが観測したのボール情報
  /// @param time Visionがフレームをキャプチャした時刻
  /// @return 状態推定後のボール情報
  std::optional<model::ball> update(std::optional<model::ball> ball,
                                    std::chrono::system_clock::time_point time) override;

  virtual ~ball() = default;
};
} // namespace state_observer
} // namespace filter
} // namespace ai_server

#endif // AI_SERVER_FILTER_STATE_OBSERVER_BALL_H
