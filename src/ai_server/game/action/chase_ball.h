#ifndef AI_SERVER_GAME_ACTION_CHASE_BALL_H
#define AI_SERVER_GAME_ACTION_CHASE_BALL_H

#include <Eigen/Core>

#include "ai_server/model/command.h"
#include "base.h"

namespace ai_server::game::action {

class chase_ball : public base {
public:
  /// @brief コンストラクタ
  /// @param ctx コンテキスト
  /// @param id 使うロボットのid
  /// @param target 目標とする座標(デフォルトでは敵ゴール)
  chase_ball(context& ctx, unsigned int id, const Eigen::Vector2d& target);

  /// @brief コンストラクタ
  /// @param ctx コンテキスト
  /// @param id 使うロボットのid
  chase_ball(context& ctx, unsigned int id);

  /// 動作モード
  enum class mode {
    move_to_ball, ///< first_posに移動
    wraparound,   ///< ボールに回り込む
    dribble,      ///< ドリブルしながら前進
    wait_ball     ///< ボールが来るのを待つ
  };

  /// @brief 目標位置を設定する
  /// @param x, y 目標とする座標
  void set_target(double x, double y);

  /// @brief 目標位置を設定する
  /// @param target 目標とする座標
  void set_target(const Eigen::Vector2d& target);

  /// @brief モードを取得する
  /// @return モード
  mode mode() const;

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  model::command execute() override;

  /// @return 動作が完了しているか?
  bool finished() const override;

private:
  // モード
  enum mode mode_;

  // 蹴りたい方向
  Eigen::Vector2d kick_target_;

  // mode::wait_ball 開始時のfirst_posまでの距離
  double start_dist_;

  // 移動時間のカウント [fps]
  int count_ = 0;
  // 減速時間のカウント [fps]
  int sub_count_ = 0;

  // mode::wait_ball 開始時のballの位置
  Eigen::Vector2d ball_pos_;

  //初期化
  bool init_flag_ = false;
  //ボールを待つ動作の判断
  bool wait_flag_ = false;
  //回り込みの判断
  bool wrap_flag_ = false;
  //回り込みの正負の判断
  bool sign_flag_ = false;
  //終了
  bool fin_flag_ = false;
};

} // namespace ai_server::game::action

#endif // AI_SERVER_GAME_ACTION_CHASE_BALL_H
