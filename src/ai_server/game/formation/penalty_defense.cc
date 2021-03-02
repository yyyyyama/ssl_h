#include "ai_server/game/agent/all.h"
#include "ai_server/game/agent/defense.h"
#include "ai_server/game/agent/penalty_kick.h"
#include "ai_server/util/math/distance.h"
#include "ai_server/util/math/to_vector.h"

#include "penalty_defense.h"

namespace ai_server::game::formation {
using namespace std::chrono_literals;

penalty_defense::penalty_defense(context& ctx, const std::vector<unsigned int>& ids,
                                 const unsigned int keeper_id, unsigned int enemy_keeper)
    : base(ctx),
      ids_(ids),
      keeper_id_(keeper_id),
      enemy_keeper_(enemy_keeper),
      previous_ball_(util::math::position(world().ball())),
      kicked_(false),
      finished_(false) {
  past_ball_.fill(util::math::position(world().ball()));
}

std::vector<std::shared_ptr<action::base>> penalty_defense::execute() {
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

  const auto point = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < past_ball_.size() - 1; ++i) past_ball_[i] = past_ball_[i + 1];
  past_ball_.back() = util::math::position(world().ball());

  if (!kicked_) {
    kicked_ = std::all_of(past_ball_.cbegin(), past_ball_.cend(), [this](auto&& a) {
      return util::math::distance(previous_ball_, a) > 200;
    });
    if (kicked_) kicked_time_ = point;
  }

  if (kicked_ && point - kicked_time_ > 1s) {
    finished_ = true;
  } else {
    const auto kicker_it = std::min_element(
        visible_ids.cbegin(), visible_ids.cend(), [&ball, &our_robots](auto a, auto b) {
          return util::math::distance(our_robots.at(a), ball) <
                 util::math::distance(our_robots.at(b), ball);
        });
    visible_ids.erase(std::remove(visible_ids.begin(), visible_ids.end(), *kicker_it),
                      visible_ids.end());
    auto pk = make_agent<agent::penalty_kick>(*kicker_it, visible_ids, enemy_keeper_);
    pk->set_mode(agent::penalty_kick::penalty_mode::defense);
    auto pk_action      = pk->execute();
    auto defense        = make_agent<agent::defense>(keeper_id_, std::vector<unsigned int>{});
    auto defense_action = defense->execute();
    actions.insert(actions.end(), pk_action.begin(), pk_action.end());
    actions.insert(actions.end(), defense_action.begin(), defense_action.end());
  }
  return actions;
}

bool penalty_defense ::finished() const {
  return finished_;
}

} // namespace ai_server::game::formation
