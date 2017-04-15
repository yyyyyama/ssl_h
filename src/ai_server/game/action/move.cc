#include "move.h"
#include <cmath>

namespace ai_server {
namespace game {
namespace action {

void move::move_to(double x, double y, double theta) {
  x_     = x;
  y_     = y;
  theta_ = theta;
}

model::command move::execute() {
  const double xy_allow = 10.0; //指定位置と取得した位置のズレの許容値[mm]
  const double theta_allow = 1.0 * M_PI / 180.0 //指定角度と取得した角度のズレの許容値[rad]
                             const auto this_robot_team =
                                 is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto& this_robot = this_robot_team.at(id_);

  if (std::abs(this_robot.x() - x_) <= xy_allow && std::abs(this_robot.y() - y_) <= xy_allow &&
      std::abs(this_robot.theta() - theta_) <= theta_allow) {
    //ロボットが指定位置に存在するとき
    finished_ = true;
  } else {
    //ロボットが指定位置に存在しないとき
    finished_ = false;
    model::command command(id_);
    command.set_position({x_, y_, theta_});
  }
}

bool move::finished() const {
  return finished_;
}
} // namespace action
} // namespace game
} // namespace ai_server
