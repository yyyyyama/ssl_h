#ifndef AI_SERVER_GAME_AGENT_BALL_PLACEMENT_H
#define AI_SERVER_GAME_AGENT_BALL_PLACEMENT_H

#include <map>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "ai_server/game/action/ball_place.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/receive.h"

#include "base.h"

namespace ai_server::game::agent {

class ball_placement : public base {
public:
  /// @brief コンストラクタ
  /// @param world ボールやロボットの情報
  /// @param is_yellow チームカラー(黄色か?)
  /// @param id 使うロボットのid
  /// @param target 目標座標
  /// @param is_active 自分のplacementか
  ball_placement(context& ctx, const std::vector<unsigned int>& ids,
                 const Eigen::Vector2d& target = Eigen::Vector2d(0, 0), bool is_active = true);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  std::vector<std::shared_ptr<action::base>> execute() override;

  /// @brief 目標位置を設定する
  /// @param target 目標とする座標
  void set_target(const Eigen::Vector2d& target);

  /// @brief 自分のplacementかを設定する
  /// @param is_active 自分のplacementか
  void set_active(bool is_active);

private:
  const std::vector<unsigned int> ids_;

  // ロボットが消えた時点
  std::unordered_map<unsigned int, std::chrono::steady_clock::time_point> lost_point_;
  // ロボットの座標
  std::map<unsigned int, Eigen::Vector2d> robot_pos_;
  // ロボットの速度
  std::map<unsigned int, Eigen::Vector2d> robot_vel_;
  // これ以上消えていたらlostとみなす
  std::chrono::steady_clock::duration lost_count_;
  // target of abp
  Eigen::Vector2d abp_target_;
  // 自チームのplacementか?
  bool is_active_;

  // ボールを追いかけるロボットのid
  unsigned int chaser_;
  // パスなどの待機をするロボットのid
  std::vector<unsigned int> waiters_;
  // 現在のreceiver
  unsigned int current_receiver_;

  std::map<unsigned int, std::shared_ptr<action::ball_place>> abp_;
  std::map<unsigned int, std::shared_ptr<action::receive>> receive_;
  std::map<unsigned int, std::shared_ptr<action::move>> move_;
};

} // namespace ai_server::game::agent

#endif
