#include "ai_server/game/agent/defense.h"
#include "ai_server/game/agent/kick_off.h"
#include "ai_server/util/math/distance.h"
#include "ai_server/util/math/to_vector.h"

#include "kickoff_attack.h"

namespace ai_server::game::formation {

kickoff_attack::kickoff_attack(context& ctx, const std::vector<unsigned int>& ids,
                               const unsigned int keeper_id, const bool is_start)
    : base(ctx), ids_(ids), keeper_id_(keeper_id), is_start_(is_start) {
  const auto our_robots = model::our_robots(world(), team_color());
  const auto ball       = world().ball();
  // 見えているロボットのIDを取得する
  std::vector<unsigned int> visible_ids;
  std::copy_if(
      ids_.cbegin(), ids_.cend(), std::back_inserter(visible_ids),
      [&our_robots, this](auto id) { return our_robots.count(id) && id != keeper_id_; });
  if (visible_ids.empty()) return;

  // kickerを決める
  const auto kicker_it = std::min_element(
      visible_ids.cbegin(), visible_ids.cend(), [&ball, &our_robots](auto a, auto b) {
        return util::math::distance(our_robots.at(a), ball) <
               util::math::distance(our_robots.at(b), ball);
      });
  const auto kicker = *kicker_it;
  visible_ids.erase(kicker_it);

  constexpr std::size_t default_wall_count = 0;
  std::size_t wall_count;
  if (visible_ids.size() <= 2)
    wall_count = 0; // 2台以下なら壁は無し
  else if (visible_ids.size() <= 4)
    wall_count = std::min<std::size_t>(default_wall_count, 1);
  else
    wall_count = default_wall_count;

  const auto wall_end = visible_ids.cbegin() + std::min(wall_count, visible_ids.size());
  // wall_countの値だけ前から順に壁にする
  const std::vector<unsigned int> wall(visible_ids.cbegin(), wall_end);

  visible_ids.erase(visible_ids.begin(), wall_end);

  defense_ = make_agent<agent::defense>(keeper_id_, wall);
  defense_->set_mode(agent::defense::defense_mode::normal_mode);
  kickoff_ = make_agent<agent::kick_off>(kicker, visible_ids);
  kickoff_->set_start_flag(is_start_);
}

std::vector<std::shared_ptr<action::base>> kickoff_attack::execute() {
  std::vector<std::shared_ptr<action::base>> actions;
  auto defense_action = defense_->execute();
  auto kickoff_action = kickoff_->execute();
  actions.insert(actions.end(), defense_action.begin(), defense_action.end());
  actions.insert(actions.end(), kickoff_action.begin(), kickoff_action.end());
  return actions;
}

bool kickoff_attack::finished() const {
  return kickoff_->finished();
}

} // namespace ai_server::game::formation
