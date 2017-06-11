#ifndef AI_SERVER_CONTROLLER_SLIDING_MODE_CONTROLLER_H
#define AI_SERVER_CONTROLLER_SLIDING_MODE_CONTROLLER_H

namespace ai_server {
namespace controller {
namespace detail {

/// @class  sliding mode control
/// @brief  速度生成用コントローラ
class sliding_mode_controller {
private:
  double v_target_;           // 目標指令速度
  double cycle_;              // 周期
  static const double a_max_; // 最大加速度
  static const double kp_;    // 収束速度パラメータ

public:
  /// @brief  コンストラクタ
  /// @param  cycle 制御周期
  sliding_mode_controller(double cycle);

  /// @brief  位置制御計算関数
  /// @param  delta_p  位置偏差(現在位置-目標位置)
  double control_pos(const double delta_p);

  /// @brief  速度制御計算関数
  /// @param  delta_v  速度偏差(現在速度-目標速度)
  double control_vel(const double delta_v);
};

} // detail
} // controller
} // ai_server

#endif // AI_SERVER_CONTROLLER_SLIDING_MODE_CONTROLLER_H
