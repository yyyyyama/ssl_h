#ifndef AI_SERVER_GAME_AGENT_KICK_OFF_H
#define AI_SERVER_GAME_AGENT_KICK_OFF_H

#include "base.h"
#include "ai_server/game/action/kick_action.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/no_operation.h"
#include "ai_server/game/action/receive.h"
#include "ai_server/game/action/get_ball.h"

namespace ai_server {
namespace game {
namespace agent {

class kick_off : public base {
public:
  kick_off(const model::world& world, bool is_yellow, unsigned int kicker_id,
           const std::vector<unsigned int>& waiter);
  kick_off(const model::world& world, bool is_yellow, unsigned int kicker_id);
  bool start_flag() const;
  void set_start_flag(bool start_flag);
  bool finished() const;
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  unsigned int kicker_id_;
  unsigned int receiver_id_;
  const std::vector<unsigned int> waiter_;
  bool start_flag_;
  bool evacuate_finished_;
  bool move_finished_;
  bool kick_finished_;
  bool receive_finished_;

  //移動先の座標
  double move_to_x_;
  double move_to_y_;
  double move_to_theta_;

  double ball_goal_theta_; //ボール上を軸としてゴール上を通る直線の角度 (rad)
  double move_to_robot_theta_; //最終的な移動先の地点を軸としてロボット上を通る直線の角度 (rad)
  double theta_min_; //ロボットが移動中にボールに当たらないかを判定するための値

  // action
  std::shared_ptr<action::move> move_;
  std::shared_ptr<action::kick_action> kick_;
  std::shared_ptr<action::get_ball> get_ball_;
  std::shared_ptr<action::receive> receive_;
};

} // namespace agent
} // namespace game
} // namespace ai_server
#endif
