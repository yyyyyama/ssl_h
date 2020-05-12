#ifndef AI_SERVER_GAME_ACTION_GET_BALL_H
#define AI_SERVER_GAME_ACTION_GET_BALL_H

#include <Eigen/Dense>

#include "base.h"

namespace ai_server {
namespace game {
namespace action {

class get_ball : public base {
public:
  /// @brief コンストラクタ
  /// @param world ボールやロボットの情報
  /// @param is_yellow チームカラー(黄色か?)
  /// @param id 使うロボットのid
  /// @param target 目標座標
  get_ball(context& ctx, unsigned int id, const Eigen::Vector2d& target);

  /// @brief コンストラクタ
  /// @param world ボールやロボットの情報
  /// @param is_yellow チームカラー(黄色か?)
  /// @param id 使うロボットのid
  get_ball(context& ctx, unsigned int id);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  model::command execute() override;

  /// @brief 動作の状態を表現する
  /// @var move ボール前まで移動
  /// @var dribble ボールを蹴ることができるようになるまでドリブルして蹴る
  /// @var finished 動作終了(停止)
  enum class running_state { move, dribble, finished };

  /// @brief 目標位置を設定する
  /// @param x, y 目標とする座標
  void set_target(double x, double y);

  /// @brief キックパワーを設定する
  /// @param pow キックパワー
  void set_pow(int pow);

  /// @brief チップするかを設定する
  /// @param chip チップするか
  void set_chip(bool chip);

  /// @brief 許容する目標位置とボールとの距離を設定する(キック時)
  /// @param margin 距離
  void set_kick_margin(double margin);
  void set_kick_type(const model::command::kick_flag_t& kick_type);

  /// @brief 許容する目標位置とボールとの距離を設定する(終了判定時)
  /// @param allow 距離
  void set_allow(double allow);

  /// @return 現在の目標位置
  Eigen::Vector2d target() const;

  /// @return 現在の状態
  running_state state() const;

  /// @return 動作が完了しているか?
  bool finished() const override;

private:
  // キックフラグを設定する
  void kick(const Eigen::Vector2d& robot_pos,
            const std::unordered_map<unsigned int, model::robot>& enemy_robots,
            model::command& command);
  // 状態
  running_state state_;
  // 目標
  Eigen::Vector2d target_;
  // ボール位置を一時的に保持
  Eigen::Vector2d first_ball_;
  // 目標位置との許容誤差
  double kick_margin_;
  // キックの種類・パワー
  model::command::kick_flag_t kick_type_;
  // agent指定のkick_typeをそのまま使うか
  bool manual_kick_flag_;
  // 許容誤差
  double allow_;
};
} // namespace action
} // namespace game
} // namespace ai_server

#endif
