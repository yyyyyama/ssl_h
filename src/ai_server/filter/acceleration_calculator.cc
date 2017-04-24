#include "acceleration_calculator.h"
#include <boost/math/constants/constants.hpp>

namespace ai_server {
namespace filter {

template <>
void acceleration_calculator<model::robot>::apply(
    model::robot& robot, std::chrono::high_resolution_clock::time_point time) {
  using namespace boost::math::constants;
  const auto elapsed_time = std::chrono::duration<double>(time - prev_time_);
  robot.set_ax((robot.vx() - prev_state_.vx()) / elapsed_time.count());
  robot.set_ay((robot.vy() - prev_state_.vy()) / elapsed_time.count());
  robot.set_alpha((robot.omega() - prev_state_.omega()) / elapsed_time.count());
  prev_state_ = robot;
  prev_time_  = time;
}
} // filter
} // ai_server
