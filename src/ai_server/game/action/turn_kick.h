//ペナルティキック時にrushでは単純であったため作成。

#ifndef AI_SERVER_GAME_ACTION_TURN_KICK_H
#define AI_SERVER_GAME_ACTION_TURN_KICK_H

#include <Eigen/Dense>
#include "ai_server/model/command.h"
#include "ai_server/model/ball.h"
#include "ai_server/model/field.h"
#include "ai_server/game/action/base.h"
#include "ai_server/util/math/angle.h"

namespace ai_server {
namespace game {
namespace action {
class turn_kick : public base {
private:
  model::command::kick_flag_t kick_type_;
  bool flag_;                       // ボールがキックされた時にtrue
  bool ready_flag_;                 // ボールをキックする位置についた時にture
  const Eigen::Vector2d near_post_; // 近いほうのゴールポストの座標
  const double shoot_range_;        // ゴールポストからシュート位置までの幅

public:
  turn_kick(const model::world& world, bool is_yellow, unsigned int id);
  model::command execute() override;
  bool finished() const override;
};
} // namespace action
} // namespace game
} // namespace ai_server

#endif // AI_SERVER_GAME_ACTION_TURN_KICK_H
