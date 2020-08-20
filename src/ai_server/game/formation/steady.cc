#include "steady.h"

namespace ai_server::game::formation {

steady::steady(context& ctx, const std::vector<unsigned int>& ids, unsigned int keeper_id)
    : base(ctx), steady_(ctx, ids, keeper_id) {}

std::vector<std::shared_ptr<action::base>> steady::execute() {
  return steady_.execute();
}

} // namespace ai_server::game::formation
