#ifndef AI_SERVER_GAME_AGENT_KICK_OFF_WAITER_H
#define AI_SERVER_GAME_AGENT_KICK_OFF_WAITER_H

#include <memory>
#include <vector>

#include "base.h"

namespace ai_server::game::agent {

class kick_off_waiter : public base {
public:
  /// @brief コンストラクタ
  /// @param ctx ボールやロボットの情報
  /// @param ids 使うロボットのid
  kick_off_waiter(context& ctx, const std::vector<unsigned int>& ids);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  std::vector<std::shared_ptr<action::base>> execute() override;

  enum class kickoff_mode {
    attack, ///< 攻撃側
    defense ///< 守備側
  };

  /// @brief 攻撃側か守備側かを設定する
  /// @param mode kickoff_mode
  void set_mode(const kickoff_mode& mode);

  /// @brief 現在の動作モードを取得する
  /// @return kickoff_mode
  kickoff_mode mode() const;

private:
  const std::vector<unsigned int> ids_;
  kickoff_mode mode_;
};

} // namespace ai_server::game::agent

#endif
