#include "ai_server/game/agent/all.h"
#include "ai_server/game/agent/defense.h"
#include "ai_server/util/math/distance.h"
#include "ai_server/util/math/to_vector.h"

#include "setplay_attack.h"

namespace ai_server::game::formation {
using namespace std::chrono_literals;

setplay_attack::setplay_attack(context& ctx, const std::vector<unsigned int>& ids,
                               unsigned int keeper_id)
    : base(ctx), ids_(ids), keeper_id_(keeper_id) {
  const auto our_robots = model::our_robots(world(), team_color());
  // 見えているロボットのIDを取得する
  std::vector<unsigned int> visible_ids;
  std::copy_if(ids_.cbegin(), ids_.cend(), std::back_inserter(visible_ids),
               [&our_robots](auto id) { return our_robots.count(id); });

  auto role_pair = divide_wall(visible_ids, decide_wall_count(visible_ids.size()));

  auto fw_pair = divide_kicker(role_pair.second);

  setplay_ = make_agent<agent::setplay>(fw_pair.first, fw_pair.second);
  defense_ = make_agent<agent::defense>(keeper_id_, role_pair.first);
}

std::vector<std::shared_ptr<action::base>> setplay_attack::execute() {
  std::vector<std::shared_ptr<action::base>> actions;
  auto setplay_action = setplay_->execute();
  auto defense_action = defense_->execute();
  actions.insert(actions.end(), setplay_action.begin(), setplay_action.end());
  actions.insert(actions.end(), defense_action.begin(), defense_action.end());
  return actions;
}

std::size_t setplay_attack::decide_wall_count(std::size_t num) const {
  constexpr std::size_t default_wall_count = 0;
  if (num <= 2)
    return 0; // 2台以下なら壁は無し
  else if (num <= 4)
    return std::min<std::size_t>(default_wall_count, 1);
  else
    return default_wall_count;
}

std::pair<std::vector<unsigned int>, std::vector<unsigned int>> setplay_attack::divide_wall(
    std::vector<unsigned int> visible_ids, std::size_t wall_count) const {
  // キーパーを候補から除外
  visible_ids.erase(std::remove(visible_ids.begin(), visible_ids.end(), keeper_id_),
                    visible_ids.end());
  std::vector<unsigned int> wall;
  // wall_countの値だけ前から順に壁にする
  wall.assign(visible_ids.cbegin(),
              visible_ids.cbegin() + std::min(wall_count, visible_ids.size()));

  // キーパーと壁を除いたロボットのID
  auto fw_ids = std::vector(std::min(visible_ids.cbegin() + wall_count, visible_ids.cend()),
                            visible_ids.cend());

  return std::make_pair(wall, fw_ids);
}

std::pair<unsigned int, std::vector<unsigned int>> setplay_attack::divide_kicker(
    std::vector<unsigned int>& fw_ids) const {
  const auto our_robots = model::our_robots(world(), team_color());
  const auto ball       = world().ball();
  const auto kicker_it =
      std::min_element(fw_ids.cbegin(), fw_ids.cend(), [&ball, &our_robots](auto a, auto b) {
        return util::math::distance(our_robots.at(a), ball) <
               util::math::distance(our_robots.at(b), ball);
      });
  auto waiter = fw_ids;
  waiter.erase(std::remove(waiter.begin(), waiter.end(), *kicker_it), waiter.end());
  return std::make_pair(*kicker_it, waiter);
}

bool setplay_attack::finished() const {
  return setplay_->finished();
}

} // namespace ai_server::game::formation
