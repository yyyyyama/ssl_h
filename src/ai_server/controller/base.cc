#include <limits>
#include "base.h"

namespace ai_server {
namespace controller {

base::base() : velocity_limit_(std::numeric_limits<double>::max()), stable_flag_(false) {}

base::base(const double limit) : velocity_limit_(limit), stable_flag_(false) {}

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

void base::set_stable(const bool stable) {
  stable_flag_ = stable;
}

} // namespace controller
} // namespace ai_server
