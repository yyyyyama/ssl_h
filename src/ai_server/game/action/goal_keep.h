#ifndef AI_SERVER_GAME_ACTION_GOAL_KEEP_H
#define AI_SERVER_GAME_ACTION_GOAL_KEEP_H

#include <Eigen/Dense>

#include "base.h"

namespace ai_server::game::action {
class goal_keep : public base {
public:
  goal_keep(const model::world& world, bool is_yellow, unsigned int id);
  void set_kick_type(const model::command::kick_flag_t& kick_type);
  model::command::kick_flag_t kick_type() const;
  void set_dribble(int dribble);
  int dribble() const;
  void set_halt(bool halt_flag);
  model::command execute() override;
  bool finished() const override;

private:
  int dribble_;
  model::command::kick_flag_t kick_type_;
  bool halt_flag_;
};
} // namespace ai_server::game::action
#endif
