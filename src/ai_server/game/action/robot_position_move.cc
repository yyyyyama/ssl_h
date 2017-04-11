#include "robot_position_move.h"

namespace ai_server {
namespace game {
namespace action {

//(3)
void robot_position_move::move_to(double x, double y, double theta) {
  x_     = x;
  y_     = y;
  theta_ = theta;
}

//(4)
model::command robot_position_move::execute() {}

//(5)
bool robot_position_move::finished() const {
  return finished_;
}
}
}
}
