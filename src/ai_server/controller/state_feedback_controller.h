#ifndef AI_SERVER_CONTROLLER_STATE_FEEDBACK_CONTROLLER_H
#define AI_SERVER_CONTROLLER_STATE_FEEDBACK_CONTROLLER_H

#include <memory>
#include <vector>

#include "ai_server/controller/detail/smith_predictor.h"
#include "ai_server/controller/detail/sliding_mode_controller.h"
#include "base.h"

namespace ai_server {
namespace controller {

class state_feedback_controller : public base {
private:
  double cycle_;                         // 制御周期
  const static double k_;                // 極,収束の速さ
  static const double zeta_;             // モデルパラメータζ
  static const double omega_;            // モデルパラメータω
  double kp_[2];                         // 比例ゲイン(xy,rotate)
  double ki_[2];                         // 積分ゲイン(xy,rotate)
  double kd_[2];                         // 微分ゲイン(xy,rotate)
  model::robot estimated_robot_;         // 推定ロボット状態
  velocity_t up_[2];                     // 操作量(比例,1フレーム前まで)
  velocity_t ui_[2];                     // 操作量(積分,1フレーム前まで)
  velocity_t ud_[2];                     // 操作量(微分,1フレーム前まで)
  velocity_t u_[2];                      // 操作量(1フレーム前まで)
  velocity_t e_[2];                      // 偏差(1フレーム前まで)
  const static double max_velocity_;     // 最大速度
  const static double max_acceleration_; // 最大加速度
  const static double min_acceleration_; // 最小加速度
  const static double reach_speed_;      // 加速度上昇の速さ
  detail::sliding_mode_controller sliding_mode_[2]={cycle_,cycle_};
  detail::smith_predictor smith_predictor_;

  // レギュレータ部
  void calculate_regulator(const model::robot& robot);

  // フィールド基準座標系からロボット基準座標系に変換
  position_t convert(const position_t pos, const double robot_theta);
  velocity_t convert(const velocity_t vel, const double robot_theta);

public:
  // コンストラクタ
  explicit state_feedback_controller(double cycle);

  // 制御入力更新関数
  velocity_t update(const model::robot& robot, const position_t& setpoint);
  velocity_t update(const model::robot& robot, const velocity_t& setpoint);
};

} // controller
} // ai_server

#endif // AI_SERVER_CONTROLLER_STATE_FEEDBACK_CONTROLLER_H
