#ifndef AI_SERVER_GAME_CAPTAIN_BASE_H
#define AI_SERVER_GAME_CAPTAIN_BASE_H

#include <memory>

#include "ai_server/game/formation/base.h"
#include "ai_server/model/refbox.h"

namespace ai_server::game::captain {

class base {
public:
  base(context& ctx, const model::refbox& refbox) : ctx_{ctx}, refbox_{refbox} {}

  virtual ~base() = default;

  /// @brief       呼び出されたループでの Formation を取得する
  virtual std::shared_ptr<formation::v2::base> execute() = 0;

protected:
  /// @brief       Formation を初期化するためのヘルパ関数
  /// @param args  Formation のコンストラクタに渡す引数
  template <class T, class... Args>
  std::shared_ptr<T> make_formation(Args&&... args) {
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

  const model::refbox& refbox() const {
    return refbox_;
  }

private:
  context& ctx_;
  const model::refbox& refbox_;
};

} // namespace ai_server::game::captain

#endif // AI_SERVER_GAME_CAPTAIN_BASE_H
