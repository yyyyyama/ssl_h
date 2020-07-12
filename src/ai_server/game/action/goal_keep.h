#ifndef AI_SERVER_GAME_ACTION_GOAL_KEEP_H
#define AI_SERVER_GAME_ACTION_GOAL_KEEP_H

#include <Eigen/Dense>

#include "base.h"

namespace ai_server::game::action {

class goal_keep : public base {
public:
  /// @brief コンストラクタ
  /// @param ctx コンテキスト
  /// @param id 使うロボットのid
  goal_keep(context& ctx, unsigned int id);

  /// @brief キックタイプを設定する
  /// @param kick_type キックタイプ
  void set_kick_type(const model::command::kick_flag_t& kick_type);

  /// @brief キックタイプを取得する
  /// @return キックタイプ
  model::command::kick_flag_t kick_type() const;

  /// @brief ドリブルパワーを設定する
  /// @param dribble ドリブルパワー
  void set_dribble(int dribble);

  /// @brief ドリブルパワーを取得する
  /// @return ドリブルパワー
  int dribble() const;

  /// @brief ロボットを止める
  /// @param ロボットを止めるか?
  void set_halt(bool halt_flag);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  model::command execute() override;

  /// @return 動作が完了しているか?
  bool finished() const override;

private:
  int dribble_;
  model::command::kick_flag_t kick_type_;
  bool halt_flag_;
};

} // namespace ai_server::game::action
#endif
