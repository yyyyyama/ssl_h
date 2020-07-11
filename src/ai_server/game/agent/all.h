#ifndef AI_SERVER_GAME_AGENT_ALL_H
#define AI_SERVER_GAME_AGENT_ALL_H

#include <memory>
#include <unordered_map>
#include <vector>
#include <Eigen/Core>

#include "ai_server/game/action/get_ball.h"
#include "ai_server/game/action/goal_keep.h"
#include "ai_server/game/action/guard.h"
#include "ai_server/game/action/marking.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/receive.h"
#include "ai_server/game/action/vec.h"
#include "ai_server/game/agent/base.h"
#include "ai_server/game/detail/mcts.h"
#include "ai_server/model/world.h"

namespace ai_server::game::agent {

class all : public base {
public:
  /// @brief コンストラクタ
  /// @param ctx ボールやロボットの情報
  /// @param ids 使うロボットのid
  all(context& ctx, const std::vector<unsigned int>& ids, unsigned int keeper_id);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  const std::vector<unsigned int> ids_;
  // pass目標
  Eigen::Vector2d pass_target_;
  unsigned int target_id_;
  // ロボットが消えた時点
  std::unordered_map<unsigned int, std::chrono::steady_clock::time_point> lost_point_;
  // ロボットの座標
  std::unordered_map<unsigned int, Eigen::Vector2d> robot_pos_;
  // ボールを追いかけるロボットのid
  unsigned int chaser_;
  // パスなどの待機をするロボットのid
  std::vector<unsigned int> waiters_;
  // MCTSを使う
  detail::mcts::evaluator evaluator_;

  std::unordered_map<unsigned int, std::shared_ptr<action::get_ball>> get_ball_;
  std::unordered_map<unsigned int, std::shared_ptr<action::receive>> receive_;
  std::unordered_map<unsigned int, std::shared_ptr<action::move>> move_;
  std::unordered_map<unsigned int, std::shared_ptr<action::vec>> vec_;
  std::unordered_map<unsigned int, std::shared_ptr<action::marking>> mark_;
  std::unordered_map<unsigned int, std::shared_ptr<action::guard>> guard_;

  // キーパー関連
  const unsigned int keeper_id_;
  std::shared_ptr<action::goal_keep> goal_keep_;
  std::shared_ptr<action::get_ball> keeper_get_ball_;
};

} // namespace ai_server::game::agent
#endif
