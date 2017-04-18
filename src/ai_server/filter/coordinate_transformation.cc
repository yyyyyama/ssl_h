#include <cmath>
#include "ai_server/util/math.h"
#include "coordinate_transformation.h"

namespace ai_server {
namespace filter {

template <>
void coordinate_transformation<model::ball>::apply(
    model::ball& ball, std::chrono::high_resolution_clock::time_point) {
  const double ballx    = ball.x();
  const double bally    = ball.y();
  const double costheta = std::cos(theta_);
  const double sintheta = std::sin(theta_);

  ball.set_x(costheta * ballx - sintheta * bally + x_);
  ball.set_y(sintheta * ballx + costheta * bally + y_);
}

template <>
void coordinate_transformation<model::robot>::apply(
    model::robot& robot, std::chrono::high_resolution_clock::time_point) {
  const double robotx   = robot.x();
  const double roboty   = robot.y();
  const double costheta = std::cos(theta_);
  const double sintheta = std::sin(theta_);

  robot.set_x(costheta * robotx - sintheta * roboty + x_);
  robot.set_y(sintheta * robotx + costheta * roboty + y_);
  robot.set_theta(util::wrap_to_2pi(robot.theta() + theta_));
}

} // filter
} // ai_server
