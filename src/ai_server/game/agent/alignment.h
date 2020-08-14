#ifndef AI_SERVER_GAME_AGENT_ALIGNMENT_H
#define AI_SERVER_GAME_AGENT_ALIGNMENT_H

#include "base.h"

namespace ai_server::game::agent {

class alignment : public base {
public:
  /// @brief コンストラクタ
  /// @param ctx ボールやロボットの情報
  /// @param ids 使うロボットのid
  alignment(context& ctx, const std::vector<unsigned int>& ids);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  const std::vector<unsigned int> ids_;
};

} // namespace ai_server::game::agent

#endif
