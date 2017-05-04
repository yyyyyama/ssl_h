#include "ai_server/game/action/vec.h"

namespace ai_server {
namespace game {
namespace action {

void vec::move_to(double vx, double vy, double omega) {
  vx_    = vx;
  vy_    = vy;
  omega_ = omega;
}

model::command vec::execute() {
  model::command command{id_};
  model::command::velocity_t robot_vec;

  robot_vec = {vx_, vy_, omega_};
  command.set_velocity(robot_vec);

  return command;
}

bool vec::finished() const {
  return flag_;
}

} // namespace action
} // namespace game
} // namespace ai_server
