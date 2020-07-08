#ifndef AI_SERVER_CONTROLLER_DETAIL_SMITH_PREDICTOR_H
#define AI_SERVER_CONTROLLER_DETAIL_SMITH_PREDICTOR_H

#include <deque>
#include <Eigen/Core>

#include "ai_server/model/robot.h"

namespace ai_server {
namespace controller {
namespace detail {

/// @class  smith_predictor
/// @brief  無駄時間補間,visionからのデータが7フレーム遅れるとし,その分を補間
class smith_predictor {
private:
  // 制御周期
  const double cycle_;
  // 二次遅れモデルパラメータ
  const double zeta_;
  const double omega_;
  // 7フレーム分の制御入力
  std::deque<Eigen::Vector3d> u_;

public:
  /// @brief  コンストラクタ
  /// @param  cycle 制御周期
  /// @param  zeta  ロボット二次遅れモデルのパラメータζ
  /// @param  omega ロボット二次遅れモデルのパラメータω
  smith_predictor(double cycle, double zeta, double omega);

  /// @brief  現在状態の推定
  /// @param  robot ロボット
  /// @param  u (前回)制御入力
  Eigen::Matrix3d interpolate(const model::robot& robot, const Eigen::Vector3d& u);
};

} // namespace detail
} // namespace controller
} // namespace ai_server

#endif // AI_SERVER_CONTROLLER_DETAIL_SMITH_PREDICTOR_H
