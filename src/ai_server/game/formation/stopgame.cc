#include <cmath>

#include "ai_server/game/agent/stopgame.h"
#include "ai_server/util/math/to_vector.h"

#include "stopgame.h"

namespace ai_server::game::formation {

stopgame::stopgame(context& ctx, const std::vector<unsigned int>& ids, unsigned int keeper_id)
    : base(ctx), ids_(ids), keeper_id_(keeper_id) {}

std::vector<std::shared_ptr<action::base>> stopgame::execute() {
  std::vector<std::shared_ptr<action::base>> actions;
  const auto our_robots = model::our_robots(world(), team_color());
  // 見えているロボットのIDを取得する
  std::vector<unsigned int> visible_ids;
  std::copy_if(ids_.cbegin(), ids_.cend(), std::back_inserter(visible_ids),
               [&our_robots](auto id) { return our_robots.count(id); });
  if (visible_ids.empty()) return actions;

  // stopgame時に見えている台数に応じて台数の割り振りを更新する
  auto wall_count           = decide_wall_count(visible_ids.size());
  auto [wall_ids, stop_ids] = divide_robots(visible_ids, wall_count);

  auto stop = make_agent<agent::stopgame>(stop_ids);
  auto df   = make_agent<agent::defense>(keeper_id_, wall_ids);
  df->set_mode(agent::defense::defense_mode::stop_mode);
  auto stop_actions = stop->execute();
  auto df_actions   = df->execute();
  actions.insert(actions.end(), stop_actions.begin(), stop_actions.end());
  actions.insert(actions.end(), df_actions.begin(), df_actions.end());
  return actions;
}

std::size_t stopgame::decide_wall_count(std::size_t num) const {
  constexpr std::size_t default_wall_count = 0;
  auto wall_count                          = default_wall_count;
  if (num <= 2)
    wall_count = 0; // 2台以下なら壁は無し
  else if (num <= 4)
    wall_count = std::min<decltype(wall_count)>(default_wall_count, 1);
  return wall_count;
}

std::pair<std::vector<unsigned int>, std::vector<unsigned int>> stopgame::divide_robots(
    std::vector<unsigned int> visible_ids, std::size_t wall_count) const {
  // キーパを候補から除外
  visible_ids.erase(std::remove(visible_ids.begin(), visible_ids.end(), keeper_id_),
                    visible_ids.end());
  // wall_countの値だけ前から順に壁にする
  std::vector<unsigned int> wall;
  wall.assign(visible_ids.cbegin(),
              visible_ids.cbegin() + std::min(wall_count, visible_ids.size()));
  // キーパーと壁を除いたロボットのID
  auto stop_ids = std::vector(std::min(visible_ids.cbegin() + wall_count, visible_ids.cend()),
                              visible_ids.cend());
  return std::make_pair(wall, stop_ids);
}

} // namespace ai_server::game::formation
