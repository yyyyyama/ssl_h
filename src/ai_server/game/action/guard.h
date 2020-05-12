#ifndef AI_SERVER_GAME_ACTION_GUARD_H
#define AI_SERVER_GAME_ACTION_GUARD_H

#include <Eigen/Dense>

#include "ai_server/model/command.h"
#include "ai_server/game/action/base.h"

namespace ai_server {
namespace game {
namespace action {
class guard : public base {
public:
  guard(context& ctx, unsigned int id);
  void move_to(double x, double y);
  void move_on(bool shift_flag);
  void set_kick_type(const model::command::kick_flag_t& kick_type);
  model::command::kick_flag_t kick_type() const;
  void set_dribble(int dribble);
  int dribble() const;
  void set_magnification(double magnification);
  void set_halt(bool halt_flag);
  model::command execute() override;
  bool finished() const override;

private:
  Eigen::Vector2d target_;
  bool shift_flag_;
  double magnification_;
  double margin_;
  double decelation_;
  int dribble_;
  model::command::kick_flag_t kick_type_;
  bool halt_flag_;
};
} // namespace action
} // namespace game
} // namespace ai_server
#endif
