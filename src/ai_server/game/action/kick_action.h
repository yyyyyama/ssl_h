#ifndef AI_SERVER_GAME_ACTION_KICK_ACTION_H
#define AI_SERVER_GAME_ACTION_KICK_ACTION_H

#include "ai_server/model/command.h"
#include "base.h"

namespace ai_server {
namespace game {
namespace action {

class kick_action : public base {
public:
  kick_action(const model::world& world, bool is_yellow, unsigned int id);
  void kick_to(double x, double y);

  void set_kick_type(const model::command::kick_flag_t& kick_type);

  model::command execute() override;

  bool finished() const override;

private:
  double x_;
  double y_;
  double old_ball_x;
  double old_ball_y;
  model::command::kick_flag_t kick_type_;
  bool exeflag_ = false;
};
} // namespace action
} // namespace game
} // namespace ai_server
#endif
