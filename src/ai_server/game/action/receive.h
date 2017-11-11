#ifndef AI_SERVER_GAME_ACTION_RECEIVE_H
#define AI_SERVER_GAME_ACTION_RECEIVE_H

#include "ai_server/model/command.h"
#include "ai_server/game/action/base.h"
#include <Eigen/Core>

namespace ai_server {
namespace game {
namespace action {
class receive : public base {
public:
  receive(const model::world& world, bool is_yellow, unsigned int id);
  void set_dribble(int dribble);
  int dribble();
  void set_passer(unsigned int passer_id);
  unsigned int passer();
  model::command execute() override;
  bool finished() const override;
  void set_shoot(Eigen::Vector2d shoot_pos);
  void set_kick_type(const model::command::kick_flag_t& kick_type);

private:
  int dribble_;
  unsigned int passer_id_;
  bool flag_       = false;
  bool shoot_flag_ = false;
  model::command::kick_flag_t kick_type_;
  Eigen::Vector2d shoot_pos_;
  Eigen::Vector2d dummy_pos_;
  bool approaching_flag_ = false;
};
}
}
}
#endif // AI_SERVER_GAME_ACTION_RECEIVE_H
