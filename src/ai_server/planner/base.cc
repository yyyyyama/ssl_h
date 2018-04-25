#include "base.h"

namespace ai_server {
namespace planner {

base::base() {}

base::~base() {}

position_t base::target() {
  return target_;
}

} // namespace planner
} // namespace ai_server
