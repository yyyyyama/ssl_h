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
  using base::base;
  void move_to(double x, double y, double theta);
	void set_kick_type(const model::command::kick_flag_t& kick_type);
	model::command::kick_flag_t kick_type();
	void set_dribble(int dribble);
	int dribble();
  model::command execute() override;
  bool finished() const override;

private:
  Eigen::Vector2d pos_{0.0, 0.0};
  double theta_ = 0.0;
	int dribble_;
	model::command::kick_flag_t kick_type_ = {model::command::kick_type_t::none,0.0};
  bool flag_    = false;
};
}
}
}
#endif
