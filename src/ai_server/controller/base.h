#ifndef AI_SERVER_CONTROLLER_BASE_H
#define AI_SERVER_CONTROLLER_BASE_H

#include <boost/variant.hpp>

#include "ai_server/model/command.h"
#include "ai_server/model/robot.h"

namespace ai_server {
namespace controller {

using position_t = model::command::position_t;
using velocity_t = model::command::velocity_t;

class base : public boost::static_visitor<velocity_t> {
public:
  base();
  base(const double limit);
  virtual ~base() = default;

  velocity_t operator()(const model::robot& robot, const position_t& setpoint);
  velocity_t operator()(const model::robot& robot, const velocity_t& setpoint);

  double velocity_limit() const;

  virtual void set_velocity_limit(const double limit);

protected:
  double velocity_limit_; // 制限速度

  virtual velocity_t update(const model::robot& robot, const position_t& setpoint) = 0;
  virtual velocity_t update(const model::robot& robot, const velocity_t& setpoint) = 0;
};

} // namespace controller
} // namespace ai_server

#endif // AI_SERVER_CONTROLLER_BASE_H
