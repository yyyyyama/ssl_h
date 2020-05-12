#ifndef AI_SERVER_GAME_FORMATION_BASE_H
#define AI_SERVER_GAME_FORMATION_BASE_H

#include <memory>
#include <vector>

#include "ai_server/game/agent/base.h"
#include "ai_server/game/context.h"
#include "ai_server/model/world.h"
#include "ai_server/model/refbox.h"

namespace ai_server {
namespace game {
namespace formation {

class base {
public:
  base(context& ctx, const model::refbox& refcommand);

  virtual ~base() = default;

  /// @brief                  呼び出されたループでのAgentを取得する
  virtual std::vector<std::shared_ptr<agent::base>> execute() = 0;

protected:
  /// @brief                  Agentを初期化するためのヘルパ関数
  /// @param args             Agentのコンストラクタに渡す引数
  template <class T, class... Args>
  std::shared_ptr<T> make_agent(Args&&... args) {
    return std::make_shared<T>(ctx_.world, ctx_.team_color == model::team_color::yellow,
                               std::forward<Args>(args)...);
  }

  const model::world& world() const {
    return ctx_.world;
  }

  model::team_color team_color() const {
    return ctx_.team_color;
  }

private:
  context& ctx_;

protected:
  const model::refbox& refcommand_;
};

} // namespace formation
} // namespace game
} // namespace ai_server

#endif // AI_SERVER_GAME_FORMATION_BASE_H
