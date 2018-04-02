#ifndef AI_SERVER_GAME_FORMATION_BASE_H
#define AI_SERVER_GAME_FORMATION_BASE_H

#include <memory>
#include <vector>

#include "ai_server/game/agent/base.h"
#include "ai_server/model/world.h"
#include "ai_server/model/refbox.h"

namespace ai_server {
namespace game {
namespace formation {

class base {
public:
  /// @param world            WorldModelの参照
  /// @param is_yellow        チームカラーは黄色か
  base(const model::world& world, bool is_yellow, const model::refbox& refcommand);

  virtual ~base() = default;

  /// @brief                  呼び出されたループでのAgentを取得する
  virtual std::vector<std::shared_ptr<agent::base>> execute() = 0;

protected:
  const model::world& world_;
  bool is_yellow_;
  const model::refbox& refcommand_;
};

} // namespace formation
} // namespace game
} // namespace ai_server

#endif // AI_SERVER_GAME_FORMATION_BASE_H
