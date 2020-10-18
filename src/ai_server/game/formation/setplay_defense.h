#ifndef AI_SERVER_GAME_FORMATION_SETPLAY_DEFENSE_H
#define AI_SERVER_GAME_FORMATION_SETPLAY_DEFENSE_H

#include <array>
#include <chrono>
#include <vector>
#include <Eigen/Core>

#include "ai_server/game/agent/setplay.h"

#include "base.h"

namespace ai_server::game::formation {

class setplay_defense : public v2::base {
public:
  /// @brief コンストラクタ
  /// @param ctx ボールやロボットの情報
  /// @param ids 使うロボットのid
  /// @param keeper_id キーパーのid
  setplay_defense(context& ctx, const std::vector<unsigned int>& ids,
                  const unsigned int keeper_id);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  std::vector<std::shared_ptr<action::base>> execute() override;

  /// @brief 動作が終了しているかを取得する
  /// @return 動作が終了していればtrue，していなければfalse
  bool finished() const;

private:
  std::vector<unsigned int> ids_;
  const unsigned int keeper_id_;
  std::vector<unsigned int> except_keeper_;
  std::chrono::steady_clock::time_point kicked_time_;
  Eigen::Vector2d previous_ball_;
  std::array<Eigen::Vector2d, 5> past_ball_;
  bool kicked_;
  bool finished_;
};

} // namespace ai_server::game::formation

#endif
