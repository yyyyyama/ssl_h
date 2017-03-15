#include "pid_controller.h"
#include <fstream>

namespace ai_server {
namespace controller {

pid_controller::pid_controller(double cycle) {
  cycle_ = cycle;
  kp_    = 1;
  ki_    = 0.1;
  kd_    = 0.0;
  for (int i = 0; i < 2; i++) {
    up_[i] = {0.0, 0.0, 0.0};
    ui_[i] = {0.0, 0.0, 0.0};
    ud_[i] = {0.0, 0.0, 0.0};
    e_[i]  = {0.0, 0.0, 0.0};
  }
}

velocity_t pid_controller::update(const model::robot& robot, const position_t& setpoint) {
  //位置偏差
  position_t ep;
  ep.x     = setpoint.x - robot.x();
  ep.y     = setpoint.y - robot.y();
  ep.theta = setpoint.theta - robot.theta();

  //目標速度
  velocity_t set_velocity;
  set_velocity.vx    = ep.x / cycle_;
  set_velocity.vy    = ep.y / cycle_;
  set_velocity.omega = ep.theta - cycle_;

  //速度偏差
  e_[0].vx    = set_velocity.vx - robot.vx();
  e_[0].vy    = set_velocity.vy - robot.vy();
  e_[0].omega = set_velocity.omega - robot.omega();

  //制御計算
  calculate();

  return u_;
}

velocity_t pid_controller::update(const model::robot& robot, const velocity_t& setpoint) {
  //現在偏差
  e_[0].vx    = setpoint.vx - robot.vx();
  e_[0].vy    = setpoint.vy - robot.vy();
  e_[0].omega = setpoint.omega - robot.omega();

  //制御計算
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
  std::ofstream out("out_vel.csv", std::ios::app);
  out << u_.vx << "," << u_.vy << "," << u_.omega << std::endl;
}

} // controller
} // ai_server
