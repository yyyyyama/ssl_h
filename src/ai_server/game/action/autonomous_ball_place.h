#ifndef AI_SERVER_GAME_ACTION_AUTONOMOUS_BALL_PLACE_H
#define AI_SERVER_GAME_ACTION_AUTONOMOUS_BALL_PLACE_H

#include <chrono>
#include <Eigen/Dense>

#include "base.h"

namespace ai_server {
namespace game {
namespace action {

class autonomous_ball_place : public base {
public:
  /// @brief コンストラクタ
  /// @param world ボールやロボットの情報
  /// @param is_yellow チームカラー(黄色か?)
  /// @param id 使うロボットのid
  /// @param target 目標座標
  autonomous_ball_place(const model::world& world, bool is_yellow, unsigned int id,
                        Eigen::Vector2d target);

  /// @brief 動作の状態を表現する
  /// @var move ボール前まで移動
  /// @var hold ボールを持つ
  /// @var place ボールを指定位置まで運ぶ
  /// @var wait ボールの回転を止めるためドリブルバーを止めて停止
  /// @var leave ボールから離れる
  /// @var finished 動作終了(停止)
  enum class running_state { move, hold, place, wait, leave, finished };

  /// @brief 動作のモードを表現する
  /// @var pull ボールを引く(未完成)
  /// @var push ボールを押す
  /// @var pass ボールを蹴る
  enum class place_mode { pull, push, pass };

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
  // 回り込みをしているか
  bool round_flag_;
  // ボールが見えているか
  bool ball_visible_;
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
} // namespace action
} // namespace game
} // namespace ai_server

#endif
