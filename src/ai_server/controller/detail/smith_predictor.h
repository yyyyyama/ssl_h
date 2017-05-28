#ifndef AI_SERVER_CONTROLLER_SMITH_PREDICTOR_
#define AI_SERVER_CONTROLLER_SMITH_PREDICTOR_

#include <Eigen/Core>

#include "ai_server/model/command.h"
#include "ai_server/model/robot.h"

namespace ai_server {
namespace controller {
namespace detail {

using position_t     = model::command::position_t;
using velocity_t     = model::command::velocity_t;
using acceleration_t = model::command::acceleration_t;

/// @class  smith_predictor
/// @brief  無駄時間補間,visionからのデータが7フレーム遅れるとし,その分を補間
class smith_predictor {
private:
  double cycle_; // 制御周期
  // 二次遅れモデルパラメータ
  double zeta_;
  double omega_;
  Eigen::Vector3d u_[7]; // 7フレーム分の制御入力

public:
  /// @brief  コンストラクタ
  /// @param  cycle 制御周期
  /// @param  zeta  ロボット二次遅れモデルのパラメータζ
  /// @param  omega ロボット二次遅れモデルのパラメータω
  smith_predictor(const double cycle, const double zeta, const double omega);

  /// @brief  現在状態の推定
  /// @param  robot ロボット
  /// @param  u (前回)制御入力
  Eigen::Matrix3d interpolate(const model::robot& robot, const Eigen::RowVector3d u);
};

} // detail
} // controller
} // ai_server

#endif // AI_SERVER_CONTROLLER_SMITH_PREDICTOR_
