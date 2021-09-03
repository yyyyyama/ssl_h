#ifndef AI_SERVER_GAME_ACTION_CLEAR_H
#define AI_SERVER_GAME_ACTION_CLEAR_H

#include <Eigen/Core>
#include "base.h"

namespace ai_server::game::action{

class clear : public base {
    public:
     clear(context& ctx, unsigned int id);
    model::command execute() override;
    bool finished() const override;
};

}
#endif