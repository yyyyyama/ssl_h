#ifndef AI_SERVER_GAME_ACTION_RECEIVE_H
#define AI_SERVER_GAME_ACTION_RECEIVE_H

#include "ai_server/model/command.h"
#include "ai_server/game/action/base.h"

namespace ai_server {
namespace game {
namespace action {
class receive : public base {
public:
  using base::base;
  void set_dribble(int dribble);
  int dribble();
  void set_passer(unsigned int passer_id);
  unsigned int passer();
  model::command execute() override;
  bool finished() const override;

private:
  int dribble_;
  unsigned int passer_id_;
  bool flag_ = false;
};
}
}
}
#endif // AI_SERVER_GAME_ACTION_RECEIVE_H
