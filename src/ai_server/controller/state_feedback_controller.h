#ifndef AI_SERVER_CONTROLLER_STATE_FEEDBACK_CONTROLLER_H
#define AI_SERVER_CONTROLLER_STATE_FEEDBACK_CONTROLLER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "ai_server/controller/detail/smith_predictor.h"
#include "ai_server/controller/detail/velocity_generator.h"
#include "ai_server/model/world.h"
#include "base.h"

namespace ai_server {
namespace controller {

class state_feedback_controller : public base {
  /* ------------------------------------
  ロボットの状態を3*3行列で表す
         x,    vx,    ax
         y,    vy,    ay
     theta, omega, alpha

  各要素を列ベクトルで
  ------------------------------------ */

private:
  double cycle_;                    // 制御周期
  const model::world& world_;       // worldmodel
  const static double k_;           // 極,収束の速さ
  static const double zeta_;        // モデルパラメータζ
  static const double omega_;       // モデルパラメータω
  static const double v_max_;       // 最大速度
  static const double omega_max_;   // 最大加速度
  Eigen::Vector3d kp_;              // 比例ゲイン(x,y,rotate)
  Eigen::Vector3d ki_;              // 積分ゲイン(x,y,rotate)
  Eigen::Vector3d kd_;              // 微分ゲイン(x,y,rotate)
  Eigen::Matrix3d estimated_robot_; // 推定ロボット状態
  Eigen::Vector3d up_[2];           // 操作量(比例,1フレーム前まで)
  Eigen::Vector3d ui_[2];           // 操作量(積分,1フレーム前まで)
  Eigen::Vector3d ud_[2];           // 操作量(微分,1フレーム前まで)
  Eigen::Vector3d u_[2];            // 操作量(1フレーム前まで)
  Eigen::Vector3d e_[2];            // 偏差(1フレーム前まで)
  detail::velocity_generator velocity_generator_;
  detail::smith_predictor smith_predictor_;

  // レギュレータ部
  void calculate_regulator(const model::robot& robot);

  // フィールド基準座標系からロボット基準座標系に変換
  Eigen::Vector3d convert(const Eigen::Vector3d& raw, double robot_theta);

  // 出力計算及び後処理
  void calculate_output(Eigen::Vector3d target);

  // 2点と角度から,2点を通る直線と原点を通る指定角度の直線との交点を求め
  // 原点から交点までの距離を返す
  double find_cross_point(double x_1, double y_2, double angle);

public:
  // コンストラクタ
  explicit state_feedback_controller(double cycle, const model::world& world);

  void set_velocity_limit(double limit) override;

  // 制御入力更新関数
  velocity_t update(const model::robot& robot, const position_t& setpoint) override;
  velocity_t update(const model::robot& robot, const velocity_t& setpoint) override;
};

} // namespace controller
} // namespace ai_server

#endif // AI_SERVER_CONTROLLER_STATE_FEEDBACK_CONTROLLER_H
