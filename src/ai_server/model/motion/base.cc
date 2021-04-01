#include "base.h"

namespace ai_server::model::motion {

base::base(const std::string& motion_id) : motion_id_(motion_id) {}

std::string base::motion_id() {
  return motion_id_;
}

} // namespace ai_server::model::motion
