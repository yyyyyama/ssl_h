#include "robot_position_move.h"

namespace ai_server {
namespace game {
namespace action {

void robot_position_move::move_to(double x, double y, double theta) {
  x_     = x;
  y_     = y;
  theta_ = theta;
}

model::command robot_position_move::execute() override {
  finished_ = false;
  model::command command(id_);
  command.set_position({x_, y_, theta_});
  finished_ = true;
}

bool robot_position_move::finished() const override {
  return finished_;
}
} // namespace action
} // namespace game
} // namespace ai_server
