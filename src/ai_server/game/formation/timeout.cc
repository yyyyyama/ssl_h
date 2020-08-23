#include "timeout.h"

namespace ai_server::game::formation {

timeout::timeout(context& ctx, const std::vector<unsigned int>& ids)
    : base(ctx), timeout_(ctx, ids) {}

std::vector<std::shared_ptr<action::base>> timeout::execute() {
  return timeout_.execute();
}

} // namespace ai_server::game::formation
