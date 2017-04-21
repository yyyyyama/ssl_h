#ifndef AI_SERVER_CONTROLLER_PID_CONTROLLER_H
#define AI_SERVER_CONTROLLER_PID_CONTROLLER_H

#include "base.h"

namespace ai_server {
namespace controller {

class pid_controller : public base {
private:
  double cycle_;                         // 制御周期
  model::robot robot_;                   // ロボット
  const static double kp_[2];            // 比例ゲイン(xy,rotate)
  const static double ki_[2];            // 積分ゲイン(xy,rotate)
  const static double kd_[2];            // 微分ゲイン(xy,rotate)
  const static double max_velocity_;     // 最大速度
  const static double max_acceleration_; // 最大加速度
  const static double min_acceleration_; // 最小加速度
  const static double reach_speed_;      // 加速度上昇の速さ
  velocity_t up_[2];                     // 操作量(比例,1フレーム前まで)
  velocity_t ui_[2];                     // 操作量(積分,1フレーム前まで)
  velocity_t ud_[2];                     // 操作量(微分,1フレーム前まで)
  velocity_t u_[2];                      // 操作量(1フレーム前まで)
  velocity_t e_[2];                      // 偏差(1フレーム前まで)
  // 制御計算関数
  void calculate();

public:
  // コンストラクタ
  explicit pid_controller(double cycle);

  // 制御入力更新関数
  velocity_t update(const model::robot& robot, const position_t& setpoint);
  velocity_t update(const model::robot& robot, const velocity_t& setpoint);
};

} // controller
} // ai_server

#endif // AI_SERVER_CONTROLLER_PID_CONTROLLER_H