#include <cmath>
#include "coordinate_transformation.h"
#include "util/math.h"

namespace ai_server {
namespace filter {

coordinate_transformation::coordinate_transformation(double x, double y, double theta)
    : x_(x), y_(y), theta_(theta) {}

void coordinate_transformation<model::ball>::apply(
    model::ball& ball, std::chrono::high_resolution_ckock::time_point) {
  const double ballx    = ball.x();
  const double bally    = ball.y();
  const double costheta = std::cos(theta_);
  const double sintheta = std::sin(theta_);

  ball.set_x(ballx * costheta - bally * sintheta + x_);
  ball.set_y(ballx * sintheta + bally * costheta + y_);
}

void coordinate_transformation<model::robot>::apply(
    model::robot& robot, std::chrono::high_resolution_ckock::time_point) {
  const double robotx   = robot.x();
  const double roboty   = robot.y();
  const double costheta = std::cos(theta_);
  const double sintheta = std::sin(theta_);

  robot.set_x(robotx * costheta - roboty * sintheta + x_);
  robot.set_y(robotx * sintheta + roboty * costheta + y_);
  robot.set_theta(util::wrap_to_2pi(robot.theta() + theta_));
}

} // filter
} // ai_server
