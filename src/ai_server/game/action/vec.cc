#include "ai_server/game/action/vec.h"

namespace ai_server {
namespace game {
namespace action {

vec::vec(context& ctx, unsigned int id) : base(ctx, id), velocity_(Eigen::Vector3d::Zero()) {}

void vec::move_at(double vx, double vy, double omega) {
  velocity_ = {vx, vy, omega};
}

void vec::move_at(const Eigen::Vector2d& v, double omega) {
  velocity_.head<2>() = v;
  velocity_.z()       = omega;
}

void vec::move_at(const Eigen::Vector3d& v) {
  velocity_ = v;
}

Eigen::Vector3d vec::velocity() const {
  return velocity_;
}

model::command vec::execute() {
  model::command command{};

  command.set_velocity(velocity_);

  return command;
}

bool vec::finished() const {
  return false;
}

} // namespace action
} // namespace game
} // namespace ai_server
