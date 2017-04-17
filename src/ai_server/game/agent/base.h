#ifndef AI_SERVER_GAME_AGENT_BASE_H
#define AI_SERVER_GAME_AGENT_BASE_H

#include <memory>
#include <vector>

#include "ai_server/game/action/base.h"
#include "ai_server/model/world.h"

namespace ai_server {
namespace game {
namespace agent {

class base {
public:
  /// @param world            WorldModelの参照
  /// @param is_yellow        チームカラーは黄色か
  base(const model::world& world, bool is_yellow);

  virtual ~base() = default;

  /// @brief                  呼び出されたループでのActionを取得する
  virtual std::vector<std::shared_ptr<action::base>> execute() = 0;

protected:
  /// @brief                  Actionを初期化するためのヘルパ関数
  /// @param id               ロボットのID
  /// @param args             Actionのコンストラクタに渡すその他の引数
  template <class T, class... Args>
  std::shared_ptr<T> make_action(unsigned int id, Args... args) {
    return std::make_shared<T>(world_, is_yellow_, id, std::forward<Args>(args)...);
  }

  const model::world& world_;
  bool is_yellow_;
};

} // namespace agent
} // namespace game
} // namespace ai_server

#endif // AI_SERVER_GAME_AGENT_BASE_H
