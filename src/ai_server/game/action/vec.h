#ifndef AI_SERVER_GAME_ACTION_VEC_H
#define AI_SERVER_GAME_ACTION_VEC_H

#include "base.h"
#include "ai_server/model/command.h"

namespace ai_server {
namespace game {
namespace action {

class vec : public base {
public:
  using base::base;

  void move_to(double vx, double vy, double omega = 0.0);

  model::command execute() override;

  bool finished() const override;

private:
  double vx_;
  double vy_;
  double omega_;
  bool flag_ = false;
};
} // namespace action
} // namespace game
} // namespace ai_server

#endif
