#ifndef AI_SERVER_GAME_ACTION_BASE_H
#define AI_SERVER_GAME_ACTION_BASE_H

#include "ai_server/model/command.h"
#include "ai_server/model/world.h"

namespace ai_server {
namespace game {
namespace action {

class base {
public:
  /// @param world            WorldModelの参照
  /// @param is_yellow        チームカラーは黄色か
  /// @param id               操作するロボットのID
  base(const model::world& world, bool is_yellow, unsigned int id);

  /// @brief                  呼び出されたループでの目標値を返す
  virtual model::command::setpoint_t execute() = 0;

  /// @brief                  Actionが完了したか
  virtual bool finished() const = 0;

protected:
  const model::world& world_;
  bool is_yellow_;
  unsigned int id_;
};

} // namespace action
} // namespace game
} // namespace ai_server

#endif // AI_SERVER_GAME_ACTION_BASE_H
