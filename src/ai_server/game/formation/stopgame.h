#ifndef AI_SERVER_GAME_FORMATION_STOPGAME_H
#define AI_SERVER_GAME_FORMATION_STOPGAME_H

#include <memory>
#include <vector>

#include "ai_server/game/agent/defense.h"
#include "ai_server/game/agent/stopgame.h"

#include "base.h"

namespace ai_server::game::formation {

class stopgame : public v2::base {
public:
  /// @brief コンストラクタ
  /// @param ctx ボールやロボットの情報
  /// @param ids 使うロボットのid
  /// @param keeper_id キーパーのid
  stopgame(context& ctx, const std::vector<unsigned int>& ids, unsigned int keeper_id);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  // 見えている台数をもとに壁の台数を決める
  std::size_t decide_wall_count(std::size_t num) const;
  // 見えているロボットを壁とそれ以外に分ける
  std::pair<std::vector<unsigned int>, std::vector<unsigned int>> divide_robots(
      std::vector<unsigned int> visible_ids, std::size_t wall_count) const;

  std::vector<unsigned int> ids_;
  const unsigned int keeper_id_;
};

} // namespace ai_server::game::formation

#endif // AI_SERVER_GAME_FORMATION_STOPGAME_H
