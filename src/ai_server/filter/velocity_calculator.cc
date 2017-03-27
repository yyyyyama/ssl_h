#include "velocity_calculator.h"

namespace ai_server {
namespace filter {

template <>
void velocity_calculator<model::ball>::apply(
    model::ball& ball, std::chrono::high_resolution_clock::time_point time) {
  auto x = ball.x(), y = ball.y();
  while (x == ball.x() && y == ball.y())
    ;
  std::chrono::seconds elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::high_resolution_clock::now() - time);
  ball.set_vx((ball.x() - x) / elapsed_time.count());
  ball.set_vy((ball.y() - y) / elapsed_time.count());
}

template <>
void velocity_calculator<model::robot>::apply(
    model::robot& robot, std::chrono::high_resolution_clock::time_point time) {
  auto x = robot.x(), y = robot.y(), theta = robot.theta();
  while (x == robot.x() && y == robot.y() && theta == robot.theta())
    ;
  std::chrono::seconds elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::high_resolution_clock::now() - time);
  robot.set_vx((robot.x() - x) / elapsed_time.count());
  robot.set_vy((robot.y() - y) / elapsed_time.count());
  robot.set_omega((robot.theta() - theta) / elapsed_time.count());
}
}
}