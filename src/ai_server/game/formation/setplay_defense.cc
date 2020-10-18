#include "ai_server/game/agent/all.h"
#include "ai_server/game/agent/defense.h"
#include "ai_server/game/agent/stopgame.h"
#include "ai_server/util/math/distance.h"
#include "ai_server/util/math/to_vector.h"

#include "setplay_defense.h"

namespace ai_server::game::formation {
using namespace std::chrono_literals;

setplay_defense::setplay_defense(context& ctx, const std::vector<unsigned int>& ids,
                                 const unsigned int keeper_id)
    : base(ctx),
      ids_(ids),
      keeper_id_(keeper_id),
      previous_ball_(util::math::position(world().ball())),
      kicked_(false),
      finished_(false) {
  past_ball_.fill(util::math::position(world().ball()));
}
std::vector<std::shared_ptr<action::base>> setplay_defense::execute() {
  std::vector<std::shared_ptr<action::base>> actions;
  const auto our_robots = model::our_robots(world(), team_color());
  // 見えているロボットのIDを取得する
  std::vector<unsigned int> visible_ids;
  std::copy_if(ids_.cbegin(), ids_.cend(), std::back_inserter(visible_ids),
               [&our_robots](auto id) { return our_robots.count(id); });
  if (visible_ids.empty()) return actions;

  // keeper_id_を除外
  visible_ids.erase(std::remove(visible_ids.begin(), visible_ids.end(), keeper_id_),
                    visible_ids.end());

  for (std::size_t i = 0; i < past_ball_.size() - 1; ++i) past_ball_[i] = past_ball_[i + 1];
  past_ball_.back() = util::math::position(world().ball());

  const auto& pb = previous_ball_;
  if (!kicked_) {
    kicked_ = std::all_of(past_ball_.cbegin(), past_ball_.cend(),
                          [&pb](auto&& a) { return util::math::distance(pb, a) > 200; });
    if (kicked_) kicked_time_ = std::chrono::steady_clock::now();
  }
  if (kicked_ && std::chrono::steady_clock::now() - kicked_time_ > 1s) finished_ = true;

  auto stop           = make_agent<agent::stopgame>(visible_ids);
  auto defense        = make_agent<agent::defense>(keeper_id_, std::vector<unsigned int>{});
  auto stop_action    = stop->execute();
  auto defense_action = defense->execute();
  actions.insert(actions.end(), stop_action.begin(), stop_action.end());
  actions.insert(actions.end(), defense_action.begin(), defense_action.end());
  return actions;
}

bool setplay_defense ::finished() const {
  return finished_;
}

} // namespace ai_server::game::formation
