#ifndef AI_SERVER_GAME_ACTION_RECEIVE_H
#define AI_SERVER_GAME_ACTION_RECEIVE_H

#include "ai_server/model/command.h"
#include "ai_server/game/action/base.h"
#include <Eigen/Core>

namespace ai_server {
namespace game {
namespace action {
class receive : public base {
public:
  /// @brief コンストラクタ
  /// @param ctx コンテキスト
  /// @param id 使うロボットのid
  receive(context& ctx, unsigned int id);

  /// @brief ドリブルパワーを設定する
  /// @param dribble ドリブルパワー
  void set_dribble(int dribble);

  /// @brief ドリブルパワーを取得する
  /// @return ドリブルパワー
  int dribble() const;

  /// @brief パスしてくる相手を設定する
  /// @param passer_id パスしてくる相手のID
  void set_passer(unsigned int passer_id);

  /// @brief パスしてくる相手を取得する
  /// @return パスしてくる相手のID
  unsigned int passer() const;

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  model::command execute() override;

  /// @brief シュートの目標位置を設定する
  /// @param shoot_pos シュートの目標位置
  void set_shoot(const Eigen::Vector2d& shoot_pos);

  /// @brief キックタイプを設定する
  /// @param kick_type キックタイプ
  void set_kick_type(const model::command::kick_flag_t& kick_type);

  /// @brief ペナルティエリアを避けるかを設定する
  /// @param avoid_penalty ペナルティエリアを避けるか(デフォルトではfalse)
  void set_avoid_penalty(bool avoid_penalty);

  /// @return 動作が完了しているか?
  bool finished() const override;

private:
  int dribble_;
  unsigned int passer_id_;
  bool flag_;
  bool shoot_flag_;
  bool kick_type_setted_;
  model::command::kick_flag_t kick_type_;
  Eigen::Vector2d shoot_pos_;
  bool avoid_penalty_;
};
} // namespace action
} // namespace game
} // namespace ai_server
#endif // AI_SERVER_GAME_ACTION_RECEIVE_H
