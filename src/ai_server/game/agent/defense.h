#ifndef AI_SERVER_GAME_AGENT_DEFENSE_H
#define AI_SERVER_GAME_AGENT_DEFENSE_H

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "ai_server/game/action/guard.h"
#include "ai_server/game/action/goal_keep.h"
#include "ai_server/game/action/get_ball.h"
#include "ai_server/game/agent/base.h"

namespace ai_server::game::agent {

class defense : public base {
public:
  struct enemy {
    //ロボットのid
    unsigned int id;
    //ロボットの位置
    Eigen::Vector2d position;
    //ロボットの角度
    double theta;
    //ソート計算した値を一時的に格納する変数
    double valuation;
    //評価点数
    unsigned int score;
  };

  /// @brief コンストラクタ
  /// @param ctx ボールやロボットの情報
  /// @param ids 使うロボットのid
  defense(context& ctx, unsigned int keeper_id, const std::vector<unsigned int>& wall_ids);
  enum class defense_mode {
    normal_mode,       //通常のディフェンス状態
    pk_normal_mode,    // pkのディフェンス状態
    pk_extention_mode, //試合時間超過pkのディフェンス状態
    stop_mode          //ストップゲームのディフェンス状態
  };

  /// @brief モードの設定
  /// @param mode defense_mode
  void set_mode(agent::defense::defense_mode mode);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  unsigned int keeper_id_;
  std::vector<unsigned int> enemy_ids_;
  const std::vector<unsigned int> wall_ids_;
  std::shared_ptr<action::goal_keep> keeper_;
  std::shared_ptr<action::get_ball> keeper_get_;
  defense_mode mode_;
  Eigen::Vector2d ball_;
};

} // namespace ai_server::game::agent
#endif // AI_SERVER_GAME_AGENT_DEFENSE_H
