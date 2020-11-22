#ifndef AI_SERVER_CONTROLLER_DETAIL_VELOCITY_GENERATOR_H
#define AI_SERVER_CONTROLLER_DETAIL_VELOCITY_GENERATOR_H

namespace ai_server {
namespace controller {
namespace detail {

/// @class  velocity_generator
/// @brief  速度生成器
class velocity_generator {
private:
  const double cycle_;              // 周期
  static const double v_max_;       // 最大速度
  static const double a_max_;       // 最大加速度
  static const double a_min_;       // 最小加速度
  static const double reach_speed_; // 最大加速度に達するときの速度
  static const double kp_;          // 収束速度パラメータ

public:
  /// @brief  コンストラクタ
  /// @param  cycle 制御周期
  velocity_generator(double cycle);

  /// @brief  位置制御計算関数
  /// @param  pre_vel  目標方向に対する前回指令値
  /// @param  delta_p  位置偏差(現在位置-目標位置)
  /// @param  stable   安定制御用(true->安定,false->通常)
  double control_pos(double pre_vel, double delta_p, bool stable) const;

  /// @brief  速度制御計算関数
  /// @param  pre_vel  目標方向に対する前回指令値
  /// @param  target   目標速度
  /// @param  stable   安定制御用(true->安定,false->通常)
  double control_vel(double pre_vel, double target, bool stable) const;

  /// @brief  最大加速度
  double a_max() const;
};

} // namespace detail
} // namespace controller
} // namespace ai_server

#endif // AI_SERVER_CONTROLLER_DETAIL_VELOCITY_GENERATOR_H
