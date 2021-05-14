#ifndef AI_SERVER_GAME_ACTION_BALL_PLACE_H
#define AI_SERVER_GAME_ACTION_BALL_PLACE_H

#include <chrono>

#include <Eigen/Dense>

#include "base.h"

namespace ai_server::game::action {

class ball_place : public base {
public:
  /// @brief コンストラクタ
  /// @param world ボールやロボットの情報
  /// @param is_yellow チームカラー(黄色か?)
  /// @param id 使うロボットのid
  /// @param target 目標座標
  ball_place(context& ctx, unsigned int id, const Eigen::Vector2d& target);

  /// @brief 動作の状態を表現する
  enum class running_state {
    move,    ///< ボール前まで移動
    hold,    ///< ボールを持つ
    place,   ///< ボールを指定位置まで運ぶ
    wait,    ///< ボールの回転を止めるためドリブルバーを止めて停止
    leave,   ///< ボールから離れる
    finished ///< 動作終了(停止)
  };

  /// @brief 動作のモードを表現する
  enum class place_mode {
    pull, ///< ボールを引く
    push, ///< ボールを押す
    pass  ///< ボールを蹴る
  };

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  model::command execute() override;

  /// @brief 現在の状態を取得する
  /// @return 状態
  running_state state() const;

  /// @brief kick_type
  /// @param kick_type
  void set_kick_type(const model::command::kick_flag_t& kick_type);

  /// @brief 動作が終了しているかを取得する
  /// @return 動作が終了していればtrue，していなければfalse
  bool finished() const override;

private:
  // 状態
  running_state state_;
  // モード
  place_mode mode_;
  // 動作終了の状態か
  bool finished_;
  // ボールを止めている状態か
  bool wait_flag_;
  // 待機を開始した時刻
  std::chrono::steady_clock::time_point begin_;
  // 一時的な目標
  Eigen::Vector2d target_;
  // 目標
  Eigen::Vector2d abp_target_;
  // 保持直前のボール位置
  Eigen::Vector2d first_ball_pos_;
  // キックの種類，強さ
  model::command::kick_flag_t kick_type_;
};

} // namespace ai_server::game::action

#endif
