#include "halt.h"

namespace ai_server {
namespace game {
namespace agent {

halt::halt(context& ctx, const std::vector<unsigned int>& ids) : base(ctx), ids_(ids) {}

std::vector<std::shared_ptr<action::base>> halt::execute() {
  std::vector<std::shared_ptr<action::base>> exe;
  std::shared_ptr<action::no_operation> nop;
  for (auto it = ids_.begin(); it != ids_.end(); ++it) {
    nop = make_action<action::no_operation>(*it);
    exe.push_back(nop);
  }

  return exe;
}
} // namespace agent
} // namespace game
} // namespace ai_server