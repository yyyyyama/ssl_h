#include "ball_placement.h"

namespace ai_server::game::formation {

ball_placement::ball_placement(context& ctx, const std::vector<unsigned int>& ids,
                               const Eigen::Vector2d& target, bool is_active)
    : base(ctx), ball_placement_(ctx, ids, target, is_active) {}

std::vector<std::shared_ptr<action::base>> ball_placement::execute() {
  return ball_placement_.execute();
}

} // namespace ai_server::game::formation
