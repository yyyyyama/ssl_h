#include "base.h"

namespace ai_server {
namespace controller {

velocity_t base::operator()(const model::robot& robot, const position_t& setpoint) {
  return update(robot, setpoint);
}

velocity_t base::operator()(const model::robot& robot, const velocity_t& setpoint) {
  return update(robot, setpoint);
}

} // namespace controller
} // namespace demoapp
