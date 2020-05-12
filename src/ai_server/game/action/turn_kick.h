//ペナルティキック時にrushでは単純であったため作成。

#ifndef AI_SERVER_GAME_ACTION_TURN_KICK_H
#define AI_SERVER_GAME_ACTION_TURN_KICK_H

#include <Eigen/Core>
#include "ai_server/model/command.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/game/action/base.h"

namespace ai_server {
namespace game {
namespace action {
class turn_kick : public base {
public:
  turn_kick(context& ctx, unsigned int id);
  model::command execute() override;
  bool finished() const override;

  // キックが出来る状態か
  bool wait() const;

  // 設定用関数
  void set_start_pos(Eigen::Vector2d start_pos);
  void set_radius(double radius);
  void set_angle(double angle);
  void set_kick(bool kick_flag);

private:
  // 動作状態の定義
  enum class running_state { set, move, rotate, wait, kick };

  // キックの種類
  model::command::kick_flag_t kick_type_;

  // actionが終了か
  bool flag_;

  // 初期位置
  Eigen::Vector2d start_pos_;

  // 過去のボール位置
  Eigen::Vector2d previous_ball_;

  // ボールからの指定された長さ
  double radius_;

  // 初期位置からの指定された角度
  double angle_;

  // kickをするか
  bool kick_flag_;

  // kickの準備ができているか
  bool wait_flag_;

  // 現在の動作状態
  running_state state_;
};
} // namespace action
} // namespace game
} // namespace ai_server

#endif // AI_SERVER_GAME_ACTION_TURN_KICK_H
