#include "velocity_calculator.h"

namespace ai_server {
namespace filter {


template <class T>
velocity_calculator<T>::velocity_calculator() : prev_ball_(), prev_robot_(){
  prev_time_ = std::chrono::high_resolution_clock::now();
}


template <>
void velocity_calculator<model::ball>::apply(
    model::ball& ball, std::chrono::high_resolution_clock::time_point time) {     
      const auto elapsed_time = std::chrono::duration<double>(time - prev_time_);
      ball.set_vx((ball.x() - prev_ball_.x()) / elapsed_time.count());
      ball.set_vy((ball.y() - prev_ball_.y()) / elapsed_time.count());

      prev_ball_ = ball;
      prev_time_ = time;
}

template <>
void velocity_calculator<model::robot>::apply(
    model::robot& robot, std::chrono::high_resolution_clock::time_point time) {
      const auto elapsed_time = std::chrono::duration<double>(time - prev_time_);
      robot.set_vx((robot.x() - prev_robot_.x()) / elapsed_time.count());
      robot.set_vy((robot.y() - prev_robot_.y()) / elapsed_time.count());
      robot.set_omega((robot.theta() - prev_robot_.theta()) / elapsed_time.count());

      prev_robot_ = robot;
      prev_time_ = time;
}


template <class T>
void velocity_calculator<T>::reset(){
  prev_ball_ = model::ball{};
  prev_robot_ = model::robot{};
}

template class velocity_calculator<model::ball>;
template class velocity_calculator<model::robot>;

}
}