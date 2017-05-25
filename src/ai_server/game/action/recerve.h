#ifndef AI_SERVER_GAME_ACTION_RECERVE_H
#define AI_SERVER_GAME_ACTION_RECERVE_H

#include "ai_server/model/command.h"
#include "ai_server/game/action/base.h"

namespace ai_server {
namespace game {
namespace action {
class recerve : public base {
public:
  using base::base;
  void set_dribble(int dribble);
  int dribble();
  model::command execute() override;
  bool finished() const override;

private:
  int dribble_;
  bool flag_ = false;
};
}
}
}
#endif // AI_SERVER_GAME_ACTION_RECERVE_H
