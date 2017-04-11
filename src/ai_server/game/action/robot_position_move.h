#ifndef AI_SERVER_GAME_ACTION_ROBOT_POSITION_MOVE_H
#define AI_SERVER_GAME_ACTION_ROBOT_POSITION_MOVE_H

#include "base.h"
#include "ai_server/model/command.h"

namespace ai_server {
namespace game {
namespace action {

// (1)
class robot_position_move : public base {
public:
  // (2)
  using base::base;

  // (3)
  void move_to(double x, double y, double theta = 0.0);

  // (4)
  model::command execute();

  // (5)
  bool finished() const;

private:
  double x_, y_, theta_;
  bool finished_ = false;
};
}
}
}

#endif
