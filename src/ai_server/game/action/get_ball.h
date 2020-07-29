#ifndef AI_SERVER_GAME_ACTION_GET_BALL_H
#define AI_SERVER_GAME_ACTION_GET_BALL_H

#include <utility>
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
  /// @param x 目標位置のx座標
  /// @param y 目標位置のy座標
  void set_target(double x, double y);

  /// @brief 目標位置を設定する
  /// @param t 目標位置の座標
  void set_target(const Eigen::Vector2d& t);

  /// @brief 許容する目標位置とボールとの距離を設定する(キック時)
  /// @param margin 距離
  void set_kick_margin(double margin);

  /// @brief キックを自動モードに設定する．
  void kick_automatically();

  /// @brief キックを自動モードに設定し，同時にキックパワーを変更する．
  /// @param pow キックパワー．この値はキックの種類に関係なく適用される．
  void kick_automatically(int pow);

  /// @brief キックを自動モードに設定し，同時にキックパワーを変更する．
  /// @param line_pow kick_type_t::line時のキックパワー
  /// @param chio_pow kick_type_t::chip時のキックパワー
  void kick_automatically(int line_pow, int chip_pow);

  /// @brief キックをマニュアルモードに設定する．
  void kick_manually();

  /// @brief キックをマニュアルモードに設定し，同時にキックパワーを変更する．
  /// @param kick_type キックパワー
  void kick_manually(int pow);

  /// @brief キックをマニュアルモードに設定し，同時にキックタイプを変更する．
  /// @param kick_type キックタイプ
  void kick_manually(model::command::kick_type_t type);

  /// @brief キックをマニュアルモードに設定し，同時にキックコマンドを変更する．
  /// @param kick_type キックフラグ
  void kick_manually(const model::command::kick_flag_t& kick);

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
  // 目標位置との許容誤差
  double kick_margin_;
  // マニュアルモードにおけるキックの種類・パワー
  model::command::kick_flag_t manual_kick_flag_;
  // 自動モードにおけるキックパワー (line時のキックパワー，chip時のキックパワー)
  std::pair<int, int> auto_kick_pow_;
  // キックをマニュアルモードにするか
  bool kick_manually_;
  // 許容誤差
  double allow_;
};

} // namespace action
} // namespace game
} // namespace ai_server

#endif
