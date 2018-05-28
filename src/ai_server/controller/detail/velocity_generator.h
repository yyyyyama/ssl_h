#ifndef AI_SERVER_CONTROLLER_DETAIL_VELOCITY_GENERATOR_H
#define AI_SERVER_CONTROLLER_DETAIL_VELOCITY_GENERATOR_H

namespace ai_server {
namespace controller {
namespace detail {

/// @class  velocity_generator
/// @brief  速度生成器
class velocity_generator {
private:
  double v_target_;                 // 目標指令速度
  double cycle_;                    // 周期
  static const double a_max_;       // 最大加速度
  static const double a_min_;       // 最小加速度
  static const double reach_speed_; // 最大加速度に達するときの速度
  static const double kp_;          // 収束速度パラメータ

public:
  /// @brief  コンストラクタ
  /// @param  cycle 制御周期
  velocity_generator(double cycle);

  /// @brief  位置制御計算関数
  /// @param  delta_p  位置偏差(現在位置-目標位置)
  /// @param  stable   安定制御用(true->安定,false->通常)
  double control_pos(const double delta_p, const bool stable);

  /// @brief  速度制御計算関数
  /// @param  target   目標速度
  /// @param  stable   安定制御用(true->安定,false->通常)
  double control_vel(const double target, const bool stable);
};

} // namespace detail
} // namespace controller
} // namespace ai_server

#endif // AI_SERVER_CONTROLLER_DETAIL_VELOCITY_GENERATOR_H
