#ifndef AI_SERVER_GAME_ACTION_GET_BALL_H
#define AI_SERVER_GAME_ACTION_GET_BALL_H

#include <Eigen/Dense>

#include "ai_server/model/command.h"
#include "base.h"

namespace ai_server {
namespace game {
namespace action {

class get_ball : public base {
public:
  get_ball(const model::world& world, bool is_yellow, unsigned int id);
  void set_target(double x, double y);
  void set_target(Eigen::Vector2d target);
  model::command execute() override;
  bool finished() const override;

private:
  Eigen::Vector2d target_; // 蹴りたい方向
  bool flag_;
};

} // namespace ai_server
} // namespace game
} // namespace action

#endif // AI_SERVER_GAME_ACTION_GET_BALL_H
