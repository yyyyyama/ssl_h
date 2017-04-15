#ifndef AI_SERVER_GAME_ACTION_MOVE_H
#define AI_SERVER_GAME_ACTION_MOVE_H

#include "base.h"
#include "ai_server/model/command.h"

namespace ai_server {
namespace game {
namespace action {

class move : public base {
public:
  using base::base;

  void move_to(double x, double y, double theta = 0.0);

  model::command execute() override;

  bool finished() const override;

private:
  double x_;
  double y_;
  double theta_;
  bool finished_ = false;
};
} // namespace action
} // namespace game
} // namespace ai_server

#endif
