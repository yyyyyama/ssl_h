#include "base.h"

namespace ai_server {
namespace planner {

base::base() {}

base::~base() {}

position_t base::target() {
  return target_;
}

double base::trajectory_length() {
  return trajectory_length_;
}

} // namespace planner
} // namespace ai_server
