#ifndef AI_SERVER_GAME_AGENT_BALL_PLACEMENT_H
#define AI_SERVER_GAME_AGENT_BALL_PLACEMENT_H

#include <map>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "ai_server/game/action/autonomous_ball_place.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/vec.h"
#include "ai_server/game/action/receive.h"
#include "ai_server/game/agent/base.h"
#include "ai_server/model/world.h"
#include "ai_server/util/time.h"

namespace ai_server {
namespace game {
namespace agent {

class ball_placement : public base {
public:
  /// @brief コンストラクタ
  /// @param world ボールやロボットの情報
  /// @param is_yellow チームカラー(黄色か?)
  /// @param id 使うロボットのid
  /// @param target 目標座標
  /// @param is_active 自分のplacementか
  ball_placement(const model::world& world, bool is_yellow,
                 const std::vector<unsigned int>& ids,
                 const Eigen::Vector2d target = Eigen::Vector2d(0, 0),
                 const bool is_active         = true);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  std::vector<std::shared_ptr<action::base>> execute() override;

  /// @brief 目標位置を設定する
  /// @param target 目標とする座標
  void set_target(const Eigen::Vector2d target);

  /// @brief 自分のplacementかを設定する
  /// @param is_active 自分のplacementか
  void set_active(const bool is_active);

private:
  const std::vector<unsigned int> ids_;
  // ids_の中で見えていると判定するもの
  std::vector<unsigned int> visible_ids_;

  // ロボットが消えた時点
  std::unordered_map<unsigned int, util::time_point_type> lost_point_;
  // ロボットの座標
  std::map<unsigned int, Eigen::Vector2d> robot_pos_;
  // ロボットの速度
  std::map<unsigned int, Eigen::Vector2d> robot_vel_;
  // ロボットが消えたとするか?
  std::map<unsigned int, bool> is_lost_;
  // これ以上消えていたらlostとみなす
  util::duration_type lost_count_;
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

  std::map<unsigned int, std::shared_ptr<action::autonomous_ball_place>> abp_;
  std::map<unsigned int, std::shared_ptr<action::receive>> receive_;
  std::map<unsigned int, std::shared_ptr<action::move>> move_;
};

} // namespace agent
} // namespace game
} // namespace ai_server
#endif
