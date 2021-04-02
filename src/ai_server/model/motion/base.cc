#include "base.h"

namespace ai_server::model::motion {

base::base(std::uint8_t motion_id) : motion_id_(motion_id) {}

std::uint8_t base::motion_id() {
  return motion_id_;
}

} // namespace ai_server::model::motion
