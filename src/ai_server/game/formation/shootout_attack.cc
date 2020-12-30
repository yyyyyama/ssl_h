#include "ai_server/game/agent/all.h"
#include "ai_server/game/agent/stopgame.h"
#include "ai_server/util/math/to_vector.h"

#include "shootout_attack.h"

namespace ai_server::game::formation {

shootout_attack::shootout_attack(context& ctx, const std::vector<unsigned int>& ids,
                                 const unsigned int keeper_id)
    : base(ctx), ids_(ids), keeper_id_(keeper_id) {}

std::vector<std::shared_ptr<action::base>> shootout_attack::execute() {
  std::vector<std::shared_ptr<action::base>> actions;

  const auto our_robots = model::our_robots(world(), team_color());
  std::vector<unsigned int> visible_ids;
  std::copy_if(ids_.begin(), ids_.end(), std::back_inserter(visible_ids),
               [&our_robots](auto id) { return our_robots.count(id); });
  auto all = make_agent<agent::all>(std::vector{shootout_id_}, keeper_id_);
  return all->execute();
}

} // namespace ai_server::game::formation
