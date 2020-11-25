#ifndef AI_SERVER_GAME_FORMATION_BASE_H
#define AI_SERVER_GAME_FORMATION_BASE_H

#include <memory>
#include <vector>

#include "ai_server/game/action/base.h"
#include "ai_server/game/agent/base.h"
#include "ai_server/game/context.h"
#include "ai_server/model/world.h"
#include "ai_server/model/refbox.h"

namespace ai_server {
namespace game {
namespace formation {

inline namespace v1 {

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
    return std::make_shared<T>(ctx_, std::forward<Args>(args)...);
  }

  const model::world& world() const {
    return ctx_.world;
  }

  model::team_color team_color() const {
    return ctx_.team_color;
  }

  const game::nnabla& nnabla() const {
    return *ctx_.nnabla;
  }

private:
  context& ctx_;

protected:
  const model::refbox& refcommand_;
};

} // namespace v1

namespace v2 {

class base {
public:
  base(context& ctx) : ctx_{ctx} {}

  virtual ~base() = default;

  /// @brief        呼び出されたループでのActionを取得する
  virtual std::vector<std::shared_ptr<action::base>> execute() = 0;

protected:
  /// @brief        Agentを初期化するためのヘルパ関数
  /// @param args   Agentのコンストラクタに渡す引数
  template <class T, class... Args>
  std::shared_ptr<T> make_agent(Args&&... args) {
    return std::make_shared<T>(ctx_, std::forward<Args>(args)...);
  }

  /// @brief        Actionを初期化するためのヘルパ関数
  /// @param id     ロボットのID
  /// @param args   Actionのコンストラクタに渡すその他の引数
  template <class T, class... Args>
  std::shared_ptr<T> make_action(unsigned int id, Args&&... args) {
    return std::make_shared<T>(ctx_, id, std::forward<Args>(args)...);
  }

  const model::world& world() const {
    return ctx_.world;
  }

  model::team_color team_color() const {
    return ctx_.team_color;
  }

  const game::nnabla& nnabla() const {
    return *ctx_.nnabla;
  }

private:
  context& ctx_;
};

} // namespace v2

} // namespace formation
} // namespace game
} // namespace ai_server

#endif // AI_SERVER_GAME_FORMATION_BASE_H
