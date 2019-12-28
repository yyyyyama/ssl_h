#ifndef AI_SERVER_CONTROLLER_BASE_H
#define AI_SERVER_CONTROLLER_BASE_H

#include "ai_server/model/command.h"
#include "ai_server/model/field.h"
#include "ai_server/model/robot.h"

namespace ai_server {
namespace controller {

using position_t = model::command::position_t;
using velocity_t = model::command::velocity_t;

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

  virtual velocity_t update(const model::robot& robot, const model::field& field,
                            const position_t& setpoint) = 0;
  virtual velocity_t update(const model::robot& robot, const model::field& field,
                            const velocity_t& setpoint) = 0;

protected:
  double velocity_limit_; // 制限速度
  bool stable_flag_;      // 安定制御用flag
};

} // namespace controller
} // namespace ai_server

#endif // AI_SERVER_CONTROLLER_BASE_H
