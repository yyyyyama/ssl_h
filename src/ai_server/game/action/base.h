#ifndef AI_SERVER_GAME_ACTION_BASE_H
#define AI_SERVER_GAME_ACTION_BASE_H

#include <memory>

#include "ai_server/game/context.h"
#include "ai_server/model/command.h"
#include "ai_server/model/world.h"

namespace ai_server::planner {
class base;
}

namespace ai_server {
namespace game {
namespace action {

class base {
public:
  /// @param world            WorldModelの参照
  /// @param is_yellow        チームカラーは黄色か
  /// @param id               操作するロボットのID
  base(context& ctx, unsigned int id);

  virtual ~base() = default;

  /// @brief                  割り当てられているidを取得する
  unsigned int id() const;

  /// @brief                  呼び出されたループでのロボットの命令を取得する
  virtual model::command execute() = 0;

  /// @brief                  Actionが完了したか
  virtual bool finished() const = 0;

protected:
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
  unsigned int id_;
};

/// planner を使った処理を独自で持つ action
class self_planning_base : public base {
  using base::base;

  /// @brief                  plannerを設定する
  /// @param planner          使用するplanner
  void set_path_planner(std::unique_ptr<planner::base> planner);

  /// @brief                  plannerが設定されているか
  bool has_path_planner() const;

protected:
  std::unique_ptr<planner::base> planner_;
};

} // namespace action
} // namespace game
} // namespace ai_server

#endif // AI_SERVER_GAME_ACTION_BASE_H
