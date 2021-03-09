#ifndef AI_SERVER_FILTER_STATE_OBSERVER_ROBOT_H
#define AI_SERVER_FILTER_STATE_OBSERVER_ROBOT_H

#include "ai_server/filter/base.h"
#include "ai_server/model/robot.h"
#include <Eigen/Core>
#include <array>

namespace ai_server {
namespace filter {
namespace state_observer {
class robot : public base<model::robot, timing::manual> {
private:
  // 係数群
  static constexpr double lambda_robot_observer_ =
      8.0; // オブザーバの極方程式の解の符号反転:(s + lambda_robot_observer_)^3
  static constexpr double decay_vel_ = 1.68;         // 速度減衰係数
  static constexpr double decay_acc_ = 0.45;         // 加速度減衰係数
  std::array<Eigen::Matrix<double, 3, 1>, 2> x_hat_; // 状態変数行列:[位置, 速度, 加速度]T

  std::chrono::system_clock::time_point prev_time_; // 前回呼び出しのシステム時刻
  std::chrono::system_clock::time_point capture_time_; // カメラがロボットをキャプチャした時刻
  std::chrono::system_clock::time_point receive_time_; // Visionからの信号を受信した時刻
  std::chrono::system_clock::duration
      lost_duration_; // 見えなくなってからロストさせるまでの時間
  std::optional<model::robot> raw_value_; //観測した情報
  model::robot prev_state_;

public:
  /// @brief       コンストラクタ
  /// @param wf    値を書き込むための関数
  /// @param time  lost_duration_に設定する時間
  explicit robot(std::recursive_mutex& mutex, robot::writer_func_type wf,
                 std::chrono::system_clock::duration time);

  /// @brief       オブザーバの状態更新
  /// @param vx    制御入力 (x 軸方向の速度)
  /// @param vy    制御入力 (y 軸方向の速度)
  /// @return      オブザーバを通したロボットの情報
  void observe(double vx, double vy);

  void set_raw_value(std::optional<model::robot> value,
                     std::chrono::system_clock::time_point time) override;
};
} // namespace state_observer
} // namespace filter
} // namespace ai_server

#endif // AI_SERVER_FILTER_STATE_OBSERVER_ROBOT_H
