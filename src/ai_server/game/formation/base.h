#ifndef AI_SERVER_GAME_FORMATION_BASE_H
#define AI_SERVER_GAME_FORMATION_BASE_H

#include <memory>
#include <vector>

#include "ai_server/game/agent/base.h"
#include "ai_server/model/world.h"

namespace ai_server {
namespace game {
namespace formation {

class base {
public:
  /// @param world            WorldModelの参照
  /// @param is_yellow        チームカラーは黄色か
  base(const model::world& world, bool is_yellow);

  virtual ~base() = default;

  /// @brief                  呼び出されたループでのActionを取得する
  virtual std::vector<std::shared_ptr<agent::base>> execute() = 0;

protected:
  const model::world& world_;
  bool is_yellow_;
};

} // namespace agent
} // namespace game
} // namespace ai_server

#endif // AI_SERVER_GAME_AGENT_BASE_H
