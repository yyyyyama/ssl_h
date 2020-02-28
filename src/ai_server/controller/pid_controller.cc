#include <boost/math/constants/constants.hpp>
#include <boost/algorithm/clamp.hpp>
#include <cmath>

#include "ai_server/util/math.h"
#include "pid_controller.h"

namespace ai_server {
namespace controller {

const double pid_controller::kp_[]             = {2.0, 4.0};
const double pid_controller::ki_[]             = {0.2, 0.2};
const double pid_controller::kd_[]             = {0.0, 0.0};
const double pid_controller::max_velocity_     = 5000.0; // 最大速度
const double pid_controller::max_acceleration_ = 2000.0; // 最大加速度
const double pid_controller::min_acceleration_ = 100.0;  // 速度ゼロ時の加速度
// optimized_accelがmax_accelに到達するときのロボット速度
const double pid_controller::reach_speed_ = 1500.0;

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

const velocity_t operator*(const velocity_t& vel1, const velocity_t& vel2) {
  return {vel1.vx * vel2.vx, vel1.vy * vel2.vy, vel1.omega * vel2.omega};
}

const velocity_t operator/(const velocity_t& vel, const double& c) {
  return {vel.vx / c, vel.vy / c, vel.omega / c};
}

pid_controller::pid_controller(double cycle) : base(max_velocity_), cycle_(cycle) {
  for (int i = 0; i < 2; i++) {
    up_[i] = {0.0, 0.0, 0.0};
    ui_[i] = {0.0, 0.0, 0.0};
    ud_[i] = {0.0, 0.0, 0.0};
    u_[i]  = {0.0, 0.0, 0.0};
    e_[i]  = {0.0, 0.0, 0.0};
  }
}

void pid_controller::set_velocity_limit(const double limit) {
  base::set_velocity_limit(std::min(limit, max_velocity_));
}

velocity_t pid_controller::update(const model::robot& robot,
                                  [[maybe_unused]] const model::field& field,
                                  const position_t& setpoint) {
  robot_ = robot;
  // 位置偏差
  position_t ep;
  ep.x     = setpoint.x - robot_.x();
  ep.y     = setpoint.y - robot_.y();
  ep.theta = util::math::wrap_to_pi(setpoint.theta - robot_.theta());

  double speed     = std::hypot(ep.x, ep.y);
  double direction = util::math::wrap_to_pi(std::atan2(ep.y, ep.x) - robot_.theta());
  e_[0].vx         = speed * std::cos(direction);
  e_[0].vy         = speed * std::sin(direction);
  e_[0].omega      = ep.theta;

  // 計算用にゲイン再計算
  velocity_t kp = model::command::velocity_t{kp_[0], kp_[0], kp_[1]};
  velocity_t ki = model::command::velocity_t{ki_[0], ki_[0], ki_[1]};
  velocity_t kd = model::command::velocity_t{kd_[0], kd_[0], kd_[1]};

  // 双一次変換
  // s=(2/T)*(Z-1)/(Z+1)としてPIDcontrollerを離散化
  // C=Kp+Ki/s+Kds
  up_[0] = kp * e_[0];
  ui_[0] = cycle_ * ki * (e_[0] + e_[1]) / 2 + ui_[1];
  ud_[0] = 2 * kd * (e_[0] - e_[1]) / cycle_ - ud_[1];
  u_[0]  = up_[0] + ui_[0] + ud_[0];

  // 入力制限計算
  limitation();

  return u_[0];
}

velocity_t pid_controller::update(const model::robot& robot,
                                  [[maybe_unused]] const model::field& field,
                                  const velocity_t& setpoint) {
  robot_               = robot;
  double set_direction = std::atan2(setpoint.vy, setpoint.vx);
  double set_speed     = std::hypot(setpoint.vx, setpoint.vy);
  velocity_t set_vel;
  set_vel.vx    = set_speed * std::cos(set_direction - robot_.theta());
  set_vel.vy    = set_speed * std::sin(set_direction - robot_.theta());
  set_vel.omega = setpoint.omega;

  // 現在偏差
  e_[0].vx    = set_vel.vx - robot_.vx();
  e_[0].vy    = set_vel.vy - robot_.vy();
  e_[0].omega = set_vel.omega - robot_.omega();

  // 計算用にゲイン再計算
  velocity_t kp = model::command::velocity_t{kp_[0], kp_[0], kp_[1]};
  velocity_t ki = model::command::velocity_t{ki_[0], ki_[0], ki_[1]};
  velocity_t kd = model::command::velocity_t{kd_[0], kd_[0], kd_[1]};

  // 双一次変換
  // s=(2/T)*(Z-1)/(Z+1)としてPIDcontrollerを離散化
  // C=Kp+Ki/s+Kds
  up_[0] = kp * e_[0];
  ui_[0] = cycle_ * ki * (e_[0] + e_[1]) / 2 + ui_[1];
  ud_[0] = 2 * kd * (e_[0] - e_[1]) / cycle_ - ud_[1];
  u_[0]  = u_[1] + cycle_ * (up_[0] + ui_[0] + ud_[0]);

  // 入力制限計算
  limitation();

  return u_[0];
}

void pid_controller::limitation() {
  // 加速度，速度制限
  using boost::algorithm::clamp;
  double u_angle     = std::atan2(u_[0].vy, u_[0].vx); // 今回指令速度の方向
  double u_speed     = std::hypot(u_[0].vx, u_[0].vy); // 今回指令速度の大きさ
  double robot_speed = std::hypot(robot_.vx(), robot_.vy());
  double delta_speed = u_speed - robot_speed; // 速さ偏差(今回指令とロボット速さの差)
  // 速度に応じて加速度を変化(初動でのスリップ防止)
  // 制限加速度計算
  double optimized_accel =
      robot_speed * (max_acceleration_ - min_acceleration_) / reach_speed_ + min_acceleration_;
  optimized_accel = clamp(optimized_accel, min_acceleration_, max_acceleration_);
  // 加速度制限
  if (delta_speed / cycle_ > optimized_accel &&
      std::abs(u_speed) > std::abs(robot_speed)) { // +制限加速度超過
    u_speed = robot_speed + (optimized_accel * cycle_);
  } else if (delta_speed / cycle_ < -optimized_accel &&
             std::abs(u_speed) > std::abs(robot_speed)) { // -制限加速度超過
    u_speed = robot_speed - (optimized_accel * cycle_);
  }
  // 速度制限
  u_speed = clamp(u_speed, 0.0, velocity_limit_);

  u_[0].vx = u_speed * std::cos(u_angle); // 成分速度再計算
  u_[0].vy = u_speed * std::sin(u_angle);

  // 値の更新
  up_[1] = up_[0];
  ui_[1] = ui_[0];
  ud_[1] = ud_[0];
  u_[1]  = u_[0];
  e_[1]  = e_[0];
}

} // namespace controller
} // namespace ai_server
