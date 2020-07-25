#ifndef AI_SERVER_GAME_FORMATION_HALT_H
#define AI_SERVER_GAME_FORMATION_HALT_H

#include <memory>
#include <vector>

#include "base.h"

namespace ai_server::game::formation {

/// 機体を全て停止させる
class halt : public v2::base {
public:
  /// @param ids 対象となる機体 ID のリスト
  halt(context& ctx, const std::vector<unsigned int>& ids);

  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  std::vector<std::shared_ptr<action::base>> actions_;
};

} // namespace ai_server::game::formation

#endif // AI_SERVER_GAME_FORMATION_HALT_H
