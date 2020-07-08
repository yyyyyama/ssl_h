#ifndef AI_SERVER_CONTROLLER_BASE_H
#define AI_SERVER_CONTROLLER_BASE_H

#include "ai_server/model/field.h"
#include "ai_server/model/robot.h"
#include "ai_server/model/setpoint/types.h"

namespace ai_server {
namespace controller {

class base {
public:
  base();
  base(const double limit);
  virtual ~base() = default;

  double velocity_limit() const;

  /// @brief                  登録されているロボットのControllerに速度制限をかける
  /// @param limit            速度の制限値
  virtual void set_velocity_limit(const double limit);

  /// @brief                  制御モードの切り替え
  /// @param stable           true->安定,false->通常
  virtual void set_stable(const bool stable);

  using result_type = std::tuple<double, double, double>;

  virtual result_type update(const model::robot& robot, const model::field& field,
                             const model::setpoint::position& position,
                             const model::setpoint::angle& angle)           = 0;
  virtual result_type update(const model::robot& robot, const model::field& field,
                             const model::setpoint::position& position,
                             const model::setpoint::velangular& velangular) = 0;

  virtual result_type update(const model::robot& robot, const model::field& field,
                             const model::setpoint::velocity& velocity,
                             const model::setpoint::angle& angle)           = 0;
  virtual result_type update(const model::robot& robot, const model::field& field,
                             const model::setpoint::velocity& velocity,
                             const model::setpoint::velangular& velangular) = 0;

protected:
  double velocity_limit_; // 制限速度
  bool stable_flag_;      // 安定制御用flag
};

} // namespace controller
} // namespace ai_server

#endif // AI_SERVER_CONTROLLER_BASE_H
