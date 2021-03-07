#ifndef AI_SERVER_CONTROLLER_DETAIL_SMITH_PREDICTOR_H
#define AI_SERVER_CONTROLLER_DETAIL_SMITH_PREDICTOR_H

#include <deque>
#include <Eigen/Core>

#include "ai_server/model/robot.h"

namespace ai_server {
namespace controller {
namespace detail {

/// @class  smith_predictor
/// @brief  無駄時間補間,visionからのデータが遅延するのでその分を補間
class smith_predictor {
private:
  // 遅延時間 [s]
  static const double delay_;
  // 制御周期
  const double cycle_;
  // 制御入力 (遅延時間分の補完用)
  std::deque<Eigen::Vector3d> u_;

public:
  /// @brief  コンストラクタ
  /// @param  cycle 制御周期
  smith_predictor(double cycle);

  /// @brief  現在状態の推定
  /// @param  robot ロボット
  /// @param  u (前回)制御入力
  Eigen::Matrix3d interpolate(const model::robot& robot, const Eigen::Vector3d& u);
};

} // namespace detail
} // namespace controller
} // namespace ai_server

#endif // AI_SERVER_CONTROLLER_DETAIL_SMITH_PREDICTOR_H
