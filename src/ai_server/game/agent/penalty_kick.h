#ifndef AI_SERVER_GAME_AGENT_PENALTY_KICK_H
#define AI_SERVER_GAME_AGENT_PENALTY_KICK_H

#include "base.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/no_operation.h"
#include "ai_server/game/action/marking.h"
#include "ai_server/game/action/kick_action.h"

#include <memory>
#include <vector>
#include <boost/variant.hpp>

namespace ai_server {
namespace game {
namespace agent {

class penalty_kick : public base {
public:
  penalty_kick(const model::world& world, bool is_yellow, unsigned int kicker_id,
               const std::vector<unsigned int>& ids);
  bool start_flag() const;
  void set_start_flag(bool start_flag);
  std::vector<std::shared_ptr<action::base>> execute() override;

  // private:
  //パラメータ計算
  void calculate();

  bool start_flag_;
  std::vector<std::shared_ptr<action::base>> exe_;
  std::shared_ptr<action::move> move_;
  std::shared_ptr<action::kick_action> kick_;

  unsigned int kicker_id_;
  // PK待機位置
  double kick_x_;
  double kick_y_;
  double kick_theta_;

  //蹴る方向
  double target_x_;
  double target_y_;
};
}
}
}

#endif