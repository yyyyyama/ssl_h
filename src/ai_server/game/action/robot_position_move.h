#ifndef AI_SERVER_GAME_ACTION_ROBOT_POSITION_MOVE_H
#define AI_SERVER_GAME_ACTION_ROBOT_POSITION_MOVE_H

#include "base.h"
#include "ai_server/model/command.h"

namespace ai_server {
namespace game {
namespace action {

class robot_position_move : public base {
public:
  using base::base;

  void move_to(double x, double y, double theta = 0.0);

  model::command execute();

  bool finished() const;

private:
  double x_, y_, theta_;
  bool finished_ = false;
  model::command::position_t next_robot_position_;
};
} // namespace action
} // namespace game
} // namespace ai_server

#endif
