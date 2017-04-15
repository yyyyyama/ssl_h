#include "robot_position_move.h"
#include <float.h>
#include <math.h>

namespace ai_server {
namespace game {
namespace action {

void robot_position_move::move_to(double x, double y, double theta) {
  x_     = x;
  y_     = y;
  theta_ = theta;
}

model::command robot_position_move::execute() {
  const auto this_robot_team = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto& this_robot     = this_robot_team.at(id_);

  if (fabs(this_robot.x() - x_) < DBL_EPSILON && fabs(this_robot.y() - y_) < DBL_EPSILON &&
      fabs(this_robot.theta() - theta_) < DBL_EPSILON) {
    //ロボットが指定位置に存在するとき
    finished_ = true;
  } else {
    //ロボットが指定位置に存在しないとき
    finished_ = false;
    model::command command(id_);
    command.set_position({x_, y_, theta_});
  }
}

bool robot_position_move::finished() const {
  return finished_;
}
} // namespace action
} // namespace game
} // namespace ai_server
