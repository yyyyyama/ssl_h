#ifndef AI_SERVER_GAME_ACTION_KICK_ACTION_H
#define AI_SERVER_GAME_ACTION_KICK_ACTION_H
#include "ai_server/model/command.h"
#include "base.h"
namespace ai_server {
namespace game {
namespace action {

class kick_action : public base {
public:
  using base::base;

  void kick_to(double x, double y);

  void set_kick_type(const model::command::kick_flag_t& kick_type);

  model::command execute();

  bool finished() const;

private:
  double ball_x_, ball_y_;
  double x_, y_;
  double dx_, dy_;
  model::command::position_t robot_pos_;
  model::command::kick_flag_t kick_type_;
  bool exeflag_ = false;
  int count_    = 0;
};
} // namespace action
} // namespace game
} // namespace ai_server
#endif
