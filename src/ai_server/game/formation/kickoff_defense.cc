#include <chrono>

#include "ai_server/game/agent/defense.h"
#include "ai_server/game/agent/kick_off_waiter.h"
#include "ai_server/util/math/distance.h"
#include "ai_server/util/math/to_vector.h"

#include "kickoff_defense.h"

namespace ai_server::game::formation {

kickoff_defense::kickoff_defense(context& ctx, const std::vector<unsigned int>& ids,
                                 const unsigned int keeper_id)
    : base(ctx),
      ids_(ids),
      keeper_id_(keeper_id),
      kicked_(false),
      previous_ball_(util::math::position(world().ball())) {
  past_ball_.fill(util::math::position(world().ball()));

  const auto our_robots = model::our_robots(world(), team_color());
  std::vector<unsigned int> visible_ids;
  std::copy_if(ids_.begin(), ids_.end(), std::back_inserter(visible_ids),
               [&our_robots](auto id) { return our_robots.count(id); });

  constexpr std::size_t default_wall_count = 0;
  std::size_t wall_count;
  if (visible_ids.size() <= 2 || visible_ids.size() > 4)
    wall_count = default_wall_count;
  else
    wall_count = std::min<std::size_t>(default_wall_count, 1);

  visible_ids.erase(std::remove(visible_ids.begin(), visible_ids.end(), keeper_id_));
  const auto wall_end = visible_ids.cbegin() + std::min(wall_count, visible_ids.size());
  const auto wall     = std::vector(visible_ids.cbegin(), wall_end);
  const auto others   = std::vector(wall_end, visible_ids.cend());

  defense_ = make_agent<agent::defense>(keeper_id_, wall);
  defense_->set_mode(agent::defense::defense_mode::normal_mode);
  kickoff_waiter_ = make_agent<agent::kick_off_waiter>(others);
  kickoff_waiter_->set_mode(agent::kick_off_waiter::kickoff_mode::defense);
}

std::vector<std::shared_ptr<action::base>> kickoff_defense::execute() {
  std::vector<std::shared_ptr<action::base>> actions;

  for (std::size_t i = 0; i < past_ball_.size() - 1; ++i) past_ball_[i] = past_ball_[i + 1];
  past_ball_.back() = util::math::position(world().ball());

  kicked_ = kicked_ || std::all_of(past_ball_.cbegin(), past_ball_.cend(), [this](auto&& a) {
              return util::math::distance(previous_ball_, a) > 200;
            });

  auto defense_action = defense_->execute();
  auto kickoff_action = kickoff_waiter_->execute();
  actions.insert(actions.end(), defense_action.begin(), defense_action.end());
  actions.insert(actions.end(), kickoff_action.begin(), kickoff_action.end());
  return actions;
}

bool kickoff_defense::finished() const {
  return kicked_;
}

} // namespace ai_server::game::formation
