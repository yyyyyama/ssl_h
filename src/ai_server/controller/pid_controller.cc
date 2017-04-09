#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "ai_server/util/math.h"
#include "pid_controller.h"

namespace ai_server {
namespace controller {

const double pid_controller::kp_ = 1.0;
const double pid_controller::ki_ = 0.1;
const double pid_controller::kd_ = 0.0;

// model::command::velocityの四則演算のオーバロード
const velocity_t operator+(const velocity_t& vel1, const velocity_t& vel2) {
  return {vel1.vx + vel2.vx, vel1.vy + vel2.vy, vel1.omega + vel2.omega};
}

const velocity_t operator-(const velocity_t& vel1, const velocity_t& vel2) {
  return {vel1.vx - vel2.vx, vel1.vy - vel2.vy, vel1.omega - vel2.omega};
}

const velocity_t operator*(const double& c, const velocity_t& vel) {
  return {c * vel.vx, c * vel.vy, c * vel.omega};
}

const velocity_t operator/(const velocity_t& vel, const double& c) {
  return {vel.vx / c, vel.vy / c, vel.omega / c};
}

pid_controller::pid_controller(double cycle) : cycle_(cycle) {
  for (int i = 0; i < 2; i++) {
    up_[i] = {0.0, 0.0, 0.0};
    ui_[i] = {0.0, 0.0, 0.0};
    ud_[i] = {0.0, 0.0, 0.0};
    e_[i]  = {0.0, 0.0, 0.0};
  }
}

velocity_t pid_controller::update(const model::robot& robot, const position_t& setpoint) {
  // 位置偏差
  position_t ep;
  ep.x     = setpoint.x - robot.x();
  ep.y     = setpoint.y - robot.y();
  ep.theta = util::wrap_to_pi(setpoint.theta - robot.theta());

  double speed     = std::hypot(ep.x, ep.y);
  double direction = util::wrap_to_pi(std::atan2(ep.y, ep.x) - robot.theta());
  e_[0].vx         = speed * std::cos(direction);
  e_[0].vy         = speed * std::sin(direction);
  e_[0].omega      = ep.theta;

  // 制御計算
  calculate();

  return u_;
}

velocity_t pid_controller::update(const model::robot& robot, const velocity_t& setpoint) {
  // 現在偏差
  e_[0].vx    = setpoint.vx - robot.vx();
  e_[0].vy    = setpoint.vy - robot.vy();
  e_[0].omega = setpoint.omega - robot.omega();

  // 制御計算
  calculate();

  return u_;
}

void pid_controller::calculate() {
  // s=(2/T)*(Z-1)/(Z+1)としてPIDcontrollerを離散化
  // C=Kp+Ki/s+Kds
  up_[0] = kp_ * e_[0];
  ui_[0] = ki_ * cycle_ * (e_[0] + e_[1]) / 2 + ui_[1];
  ud_[0] = 2 * kd_ * (e_[0] - e_[1]) / cycle_ - ud_[1];
  u_     = up_[0] + ui_[0] + ud_[0];

  up_[1] = up_[0];
  ui_[1] = ui_[0];
  ud_[1] = ud_[0];
  e_[1]  = e_[0];
}

} // controller
} // ai_server
