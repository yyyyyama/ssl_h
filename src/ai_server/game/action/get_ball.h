#ifndef AI_SERVER_GAME_ACTION_GET_BALL_H
#define AI_SERVER_GAME_ACTION_GET_BALL_H

#include <Eigen/Dense>

#include "ai_server/util/time.h"
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
  get_ball(const model::world& world, bool is_yellow, unsigned int id,
           const Eigen::Vector2d& target);

  /// @brief コンストラクタ
  /// @param world ボールやロボットの情報
  /// @param is_yellow チームカラー(黄色か?)
  /// @param id 使うロボットのid
  get_ball(const model::world& world, bool is_yellow, unsigned int id);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  model::command execute() override;

  /// @brief 動作の状態を表現する
  /// @var move ボール前まで移動
  /// @var hold ボールを持つ
  /// @var dribble ボールを蹴ることができるようになるまでドリブルして蹴る
  /// @var finished 動作終了(停止)
  enum class running_state { move, hold, dribble, finished };

  /// @brief 動作のモードを表現する
  /// @var pull ボールを引く(未完成)
  /// @var push ボールを押す
  enum class place_mode { pull, push, pass };

  /// @brief 目標位置を設定する
  /// @param x, y 目標とする座標
  void set_target(double x, double y);

  /// @brief キックパワーを設定する
  /// @param pow キックパワー
  void set_pow(double pow);

  /// @brief チップするかしないかを設定する
  /// @param chip チップするかしないか
  void set_chip(bool chip);

  /// @brief 許容する目標位置とボールとの距離を設定する(キック時)
  /// @param margin 距離
  void set_kick_margin(double margin);

  /// @brief 許容する目標位置とボールとの距離を設定する(終了判定時)
  /// @param allow 距離
  void set_allow(double allow);

  /// @return 現在の目標位置
  Eigen::Vector2d target() const;

  /// @return キックパワー
  double pow() const;

  /// @return チップフラグが立っているか?
  bool chip() const;

  /// @return 現在の状態
  running_state state() const;

  /// @return 動作が完了しているか?
  bool finished() const override;

private:
  // キックフラグを設定する
  void kick(const Eigen::Vector2d& robot_pos,
            const std::unordered_map<unsigned int, model::robot>& enemy_robots,
            model::command& command, const double pow, const bool chip);
  // 状態
  running_state state_;
  // ボールを押すか引くかのモード
  place_mode mode_;
  // 回り込みをしているか
  bool round_flag_;
  // 暫定目標
  Eigen::Vector2d target_;
  // 最終目標
  Eigen::Vector2d final_target_;
  // ボール位置を一時的に保持
  Eigen::Vector2d first_ball_;
  // 蹴る強さ
  double pow_;
  // 目標位置との許容誤差
  double kick_margin_;
  // チップキックにするかどうか
  bool chip_;
  // 許容誤差
  double allow_;
};
} // namespace action
} // namespace game
} // namespace ai_server

#endif
