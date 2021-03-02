#include "ai_server/game/agent/defense.h"
#include "ai_server/game/agent/penalty_kick.h"
#include "ai_server/util/math/distance.h"

#include "shootout_defense.h"

namespace ai_server::game::formation {

shootout_defense::shootout_defense(context& ctx, const std::vector<unsigned int>& ids,
                                   const unsigned int keeper_id,
                                   const unsigned int enemy_keeper)
    : base(ctx),
      ids_(ids),
      keeper_id_(keeper_id),
      enemy_keeper_(enemy_keeper),
      is_start_(false) {}

std::vector<std::shared_ptr<action::base>> shootout_defense::execute() {
  std::vector<std::shared_ptr<action::base>> actions;

  const auto our_robots = model::our_robots(world(), team_color());
  const auto ball       = world().ball();
  // 見えているロボットのIDを取得する
  std::vector<unsigned int> visible_ids;
  std::copy_if(ids_.cbegin(), ids_.cend(), std::back_inserter(visible_ids),
               [&our_robots](auto id) { return our_robots.count(id); });
  if (visible_ids.empty()) return actions;

  // keeper_id_を除外
  visible_ids.erase(std::remove(visible_ids.begin(), visible_ids.end(), keeper_id_),
                    visible_ids.end());
  const auto kicker_it = std::min_element(
      visible_ids.cbegin(), visible_ids.cend(), [&ball, &our_robots](auto a, auto b) {
        return util::math::distance(our_robots.at(a), ball) <
               util::math::distance(our_robots.at(b), ball);
      });

  auto pk = make_agent<agent::penalty_kick>(*kicker_it, visible_ids, enemy_keeper_);
  pk->set_mode(agent::penalty_kick::penalty_mode::defense);
  auto pk_action = pk->execute();

  auto defense = make_agent<agent::defense>(keeper_id_, std::vector<unsigned int>{});
  if (is_start_)
    defense->set_mode(agent::defense::defense_mode::pk_extention_mode);
  else
    defense->set_mode(agent::defense::defense_mode::pk_normal_mode);

  auto defense_action = defense->execute();
  actions.insert(actions.end(), pk_action.begin(), pk_action.end());
  actions.insert(actions.end(), defense_action.begin(), defense_action.end());
  return actions;
}

void shootout_defense::start() {
  is_start_ = true;
}

} // namespace ai_server::game::formation
