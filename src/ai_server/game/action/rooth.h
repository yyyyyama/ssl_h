#ifndef AI_SERVER_GAME_ACTION_ROOTH_H
#define AI_SERVER_GAME_ACTION_ROOTH_H

#include <Eigen/Core>
#include "base.h"
#include "ai_server/game/action/move_2walk.h"

namespace ai_server::game::action{

class rooth : public base {
  public:
    rooth(context& ctx, unsigned int id);
    model::command execute() override;
    bool finished() const override;

  private:
    game::action::move_2walk move_2walk_;
};

}
#endif