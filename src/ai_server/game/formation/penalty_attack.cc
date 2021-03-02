#include "ai_server/game/agent/defense.h"
#include "ai_server/game/agent/penalty_kick.h"
#include "ai_server/util/math/distance.h"
#include "ai_server/util/math/to_vector.h"

#include "penalty_attack.h"

namespace ai_server::game::formation {

penalty_attack::penalty_attack(context& ctx, const std::vector<unsigned int>& ids,
                               const unsigned int keeper_id, const unsigned int enemy_keeper,
                               const bool is_start)
    : base(ctx),
      ids_(ids),
      keeper_id_(keeper_id),
      enemy_keeper_(enemy_keeper),
      is_start_(is_start) {
  const auto our_robots = model::our_robots(world(), team_color());
  const auto ball       = world().ball();
  // 見えているロボットのIDを取得する
  std::vector<unsigned int> visible_ids;
  std::copy_if(ids_.cbegin(), ids_.cend(), std::back_inserter(visible_ids),
               [&our_robots](auto id) { return our_robots.count(id); });

  // keeper_id_を除外
  visible_ids.erase(std::remove(visible_ids.begin(), visible_ids.end(), keeper_id_),
                    visible_ids.end());
  const auto kicker_it = std::min_element(
      visible_ids.cbegin(), visible_ids.cend(), [&ball, &our_robots](auto a, auto b) {
        return util::math::distance(our_robots.at(a), ball) <
               util::math::distance(our_robots.at(b), ball);
      });
  visible_ids.erase(std::remove(visible_ids.begin(), visible_ids.end(), *kicker_it),
                    visible_ids.end());

  pk_ = make_agent<agent::penalty_kick>(*kicker_it, visible_ids, enemy_keeper_);
  pk_->set_mode(agent::penalty_kick::penalty_mode::attack);
  pk_->set_start_flag(is_start_);
  df_ = make_agent<agent::defense>(keeper_id_, std::vector<unsigned int>{});
  df_->set_mode(agent::defense::defense_mode::pk_normal_mode);
}

std::vector<std::shared_ptr<action::base>> penalty_attack::execute() {
  std::vector<std::shared_ptr<action::base>> actions;
  auto defense_action = df_->execute();
  auto pk_action      = pk_->execute();
  actions.insert(actions.end(), pk_action.begin(), pk_action.end());
  actions.insert(actions.end(), defense_action.begin(), defense_action.end());
  return actions;
}

bool penalty_attack::finished() const {
  return pk_->finished();
}

} // namespace ai_server::game::formation
