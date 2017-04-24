#ifndef AI_SERVER_CONTROLLER_SLIDING_MODE_CONTROLLER_H
#define AI_SERVER_CONTROLLER_SLIDING_MODE_CONTROLLER_H

namespace ai_server {
namespace controller {

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

  /// @brief  制御計算関数
  /// @param  delta_p  位置偏差(現在位置-目標位置)
  double control(const double delta_p);
};

} // controller
} // ai_server

#endif // AI_SERVER_CONTROLLER_SLIDING_MODE_CONTROLLER_H
