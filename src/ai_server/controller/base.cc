#include <limits>
#include "base.h"

namespace ai_server {
namespace controller {

base::base() : velocity_limit_(std::numeric_limits<double>::max()) {}

base::base(const double limit) : velocity_limit_(limit) {}

velocity_t base::operator()(const model::robot& robot, const position_t& setpoint) {
  return update(robot, setpoint);
}

velocity_t base::operator()(const model::robot& robot, const velocity_t& setpoint) {
  return update(robot, setpoint);
}

double base::velocity_limit() const {
  return velocity_limit_;
}

void base::set_velocity_limit(const double limit) {
  velocity_limit_ = limit;
}

} // namespace controller
} // namespace ai_server
