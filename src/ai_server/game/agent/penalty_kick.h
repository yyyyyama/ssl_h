#ifndef AI_SERVER_GAME_AGENT_PENALTY_KICK_H
#define AI_SERVER_GAME_AGENT_PENALTY_KICK_H

#include "base.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/turn_kick.h"

#include <memory>
#include <vector>

///
/// このagentはpenalty時にペナルティライン上からキックするルールで使用されていたものです。
/// ルール変更後は適応した別のagentを使うことを推奨します。
///

namespace ai_server::game::agent {

class penalty_kick : public base {
public:
  /// @brief コンストラクタ
  /// @param ctx ボールやロボットの情報
  /// @param kicker_id キッカーにするロボットのid
  /// @param ids キッカー以外のロボットのid
  /// @param enemy_keeper 敵キーパーのid
  penalty_kick(context& ctx, unsigned int kicker_id, const std::vector<unsigned int>& ids,
               unsigned int enemy_keeper);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  std::vector<std::shared_ptr<action::base>> execute() override;

  enum class penalty_mode {
    attack, ///< 攻撃側
    defense ///< 守備側
  };

  /// @brief 攻撃側か守備側かを設定する
  /// @param mode 設定するpenalty_mode
  void set_mode(penalty_kick::penalty_mode mode);

  /// @brief スタートしているかを設定する
  /// @param start_flag スタートしているか
  void set_start_flag(bool start_flag);

  /// @brief penalty_kickの動作が終了したか取得する
  /// @return 終了していたらtrue
  bool finished() const;

private:
  enum class attack_state { wait, check, kick };

  attack_state state_;
  penalty_mode mode_;

  bool start_flag_;
  std::vector<unsigned int> ids_;
  std::shared_ptr<action::turn_kick> turn_kick_;
  std::shared_ptr<action::move> kicker_move_;

  unsigned int kicker_id_;

  // 敵キーパーの動きを監視して蹴るタイミングを変える
  unsigned int enemy_keeper_id_;

  std::chrono::steady_clock::time_point change_command_time_; // PKの経過時間
};

} // namespace ai_server::game::agent

#endif
