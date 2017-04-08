#include "velocity_calculator.h"
#include <boost/math/constants/constants.hpp>

namespace ai_server {
namespace filter {

template <>
void velocity_calculator<model::ball>::apply(
    model::ball& ball, std::chrono::high_resolution_clock::time_point time) {
  const auto elapsed_time = std::chrono::duration<double>(time - prev_time_);
  ball.set_vx((ball.x() - prev_state_.x()) / elapsed_time.count());
  ball.set_vy((ball.y() - prev_state_.y()) / elapsed_time.count());

  prev_state_ = ball;
  prev_time_  = time;
}

template <>
void velocity_calculator<model::robot>::apply(
    model::robot& robot, std::chrono::high_resolution_clock::time_point time) {
  using namespace boost::math::constants;
  const auto elapsed_time = std::chrono::duration<double>(time - prev_time_);
  robot.set_vx((robot.x() - prev_state_.x()) / elapsed_time.count());
  robot.set_vy((robot.y() - prev_state_.y()) / elapsed_time.count());
  if (std::abs(robot.theta() - prev_state_.theta()) <= pi<double>()) {
    robot.set_omega((robot.theta() - prev_state_.theta()) / elapsed_time.count());
  } else {
    if ((robot.theta() - prev_state_.theta()) < 0) {
      robot.set_omega((two_pi<double>() - std::fabs(robot.theta() - prev_state_.theta())) /
                      elapsed_time.count());
    } else {
      robot.set_omega(-(two_pi<double>() - std::fabs(robot.theta() - prev_state_.theta())) /
                      elapsed_time.count());
    }
  }
  prev_state_ = robot;
  prev_time_  = time;
}
}
}