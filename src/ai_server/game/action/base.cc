#include "ai_server/planner/base.h"
#include "base.h"

namespace ai_server {
namespace game {
namespace action {

base::base(context& ctx, unsigned int id) : ctx_(ctx), id_(id) {}

unsigned int base::id() const {
  return id_;
}

void self_planning_base::set_path_planner(std::unique_ptr<planner::base> planner) {
  planner_ = std::move(planner);
}

bool self_planning_base::has_path_planner() const {
  return static_cast<bool>(planner_);
}
} // namespace action
} // namespace game
} // namespace ai_server
