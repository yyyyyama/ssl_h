#ifndef AI_SERVER_GAME_ACTION_NO_OPERATION_H
#define AI_SERVER_GAME_ACTION_NO_OPERATION_H

#include "base.h"

namespace ai_server {
namespace game {
namespace action {

class no_operation : public base {
public:
  using base::base;

  model::command execute() override;

  bool finished() const override;
};
} // namespace action
} // namespace game
} // namespace ai_server

#endif