//ペナルティキック時にkickでは遅いため作成。
//キックフラグを立てた状態でボールに向かって進む。
//ボールが前回値から移動したと判断したときアクションを終了する。

#ifndef AI_SERVER_GAME_ACTION_RUSH_H
#define AI_SERVER_GAME_ACTION_RUSH_H

#include "ai_server/model/command.h"
#include "ai_server/model/ball.h"
#include "ai_server/game/action/base.h"

namespace ai_server {
namespace game {
namespace action {
class rush : public base {
public:
  rush(context& ctx, unsigned int id);
  void set_kick_type(const model::command::kick_flag_t& kick_type);
  model::command execute() override;
  bool finished() const override;

private:
  model::command::kick_flag_t kick_type_;
  bool flag_;
  model::ball previous_kick_ball_;
};
} // namespace action
} // namespace game
} // namespace ai_server
#endif
