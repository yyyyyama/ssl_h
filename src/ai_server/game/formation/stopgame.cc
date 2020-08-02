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
  const auto num                           = visible_ids.size();
  constexpr std::size_t default_wall_count = 0;
  auto wall_count                          = default_wall_count;
  if (num <= 2)
    wall_count = 0; // 2台以下なら壁は無し
  else if (num <= 4)
    wall_count = std::min(static_cast<int>(default_wall_count), 1);

  // 壁が見えているか判定　壁が見えていて、数が足りているとき作り直さない
  std::vector<unsigned int> stop_ids;
  if (wall_count != wall_.size() ||
      std::any_of(wall_.cbegin(), wall_.cend(), [&visible_ids](auto&& id) {
        return std::all_of(visible_ids.cbegin(), visible_ids.cend(),
                           [&id](auto v) { return v != id; });
      })) {
    // キーパーを候補から除外
    auto tmp_ids = visible_ids;
    tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), keeper_id_), tmp_ids.end());

    // wall_countの値だけ前から順に壁にする
    wall_.assign(tmp_ids.cbegin(), tmp_ids.cbegin() + std::min(wall_count, tmp_ids.size()));

    // キーパーと壁を除いたロボットのID
    stop_ids =
        std::vector(std::min(tmp_ids.cbegin() + wall_count, tmp_ids.cend()), tmp_ids.cend());
  }

  auto stop = make_agent<agent::stopgame>(stop_ids);
  auto df   = make_agent<agent::defense>(keeper_id_, wall_);
  df->set_mode(agent::defense::defense_mode::stop_mode);
  auto stop_actions = stop->execute();
  auto df_actions   = df->execute();
  actions.insert(actions.end(), stop_actions.begin(), stop_actions.end());
  actions.insert(actions.end(), df_actions.begin(), df_actions.end());
  return actions;
}

} // namespace ai_server::game::formation
