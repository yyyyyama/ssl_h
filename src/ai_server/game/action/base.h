#ifndef AI_SERVER_GAME_ACTION_BASE_H
#define AI_SERVER_GAME_ACTION_BASE_H

#include "ai_server/model/command.h"
#include "ai_server/model/world.h"
#include "ai_server/planner/base.h"

namespace ai_server {
namespace game {
namespace action {

class base {
public:
  /// @param world            WorldModelの参照
  /// @param is_yellow        チームカラーは黄色か
  /// @param id               操作するロボットのID
  base(const model::world& world, bool is_yellow, unsigned int id);

  virtual ~base() = default;

  /// @brief                  割り当てられているidを取得する
  unsigned int id() const;

  /// @brief                  呼び出されたループでのロボットの命令を取得する
  virtual model::command execute() = 0;

  /// @brief                  Actionが完了したか
  virtual bool finished() const = 0;

  /// @brief                  plannerを設定する
  /// @param planner          使用するplanner
  void set_path_planner(std::unique_ptr<planner::base> planner);

  /// @brief                  plannerが設定されているか
  bool has_path_planner() const;

protected:
  const model::world& world_;
  bool is_yellow_;
  unsigned int id_;
  std::unique_ptr<planner::base> planner_;
};

} // namespace action
} // namespace game
} // namespace ai_server

#endif // AI_SERVER_GAME_ACTION_BASE_H
