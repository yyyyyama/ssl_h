#ifndef AI_SERVER_GAME_AGENT_KICK_OFF_H
#define AI_SERVER_GAME_AGENT_KICK_OFF_H

#include "base.h"
#include "ai_server/game/action/kick_action.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/no_operation.h"

namespace ai_server {
namespace game {
namespace agent {

class kick_off : public base {
public:
  kick_off(const model::world& world, bool is_yellow, unsigned int kicker_id);
  bool start_flag() const;
  void set_start_flag(bool start_flag);
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  unsigned int kicker_id_;
  bool start_flag_    = false;
  bool move_finished_ = false;
  bool kick_finished_ = false;

  //移動先の座標
  double move_to_x_;
  double move_to_y_;
  double move_to_theta_;

  //ロボットが移動中にボールに当たらないかを判定するための値
  double move_to_robot_theta_;

  double ball_goal_theta_; //ボール上を軸としてゴール上を通る直線の角度 (rad)
  double theta_min_;
  // action
  std::shared_ptr<action::move> move;
  std::shared_ptr<action::kick_action> kick;
  // std::shared_ptr<action::no_operation> no_op;
};

} // namespace agent
} // namespace game
} // namespace ai_server
#endif
