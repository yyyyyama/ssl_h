#include <boost/algorithm/clamp.hpp>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <Eigen/Core>

#include "ai_server/util/math.h"
#include "state_feedback_controller.h"

namespace ai_server {
namespace controller {

using boost::algorithm::clamp;
using boost::math::constants::half_pi;
using boost::math::constants::pi;

const double state_feedback_controller::k_                = 49.17;
const double state_feedback_controller::zeta_             = 1.0;
const double state_feedback_controller::omega_            = 49.17;
const double state_feedback_controller::max_velocity_     = 5000.0; // 最大速度
const double state_feedback_controller::max_acceleration_ = 3500.0; // 最大加速度
const double state_feedback_controller::min_acceleration_ = 2500.0; // 速度ゼロ時の加速度
// optimized_accelがmax_accelに到達するときのロボット速度
const double state_feedback_controller::reach_speed_ = 1000.0;

// model::command::positionの四則演算のオーバロード
const position_t operator+(const position_t& pos1, const position_t& pos2) {
  return {pos1.x + pos2.x, pos1.y + pos2.y, pos1.theta + pos2.theta};
}

const position_t operator-(const position_t& pos1, const position_t& pos2) {
  return {pos1.x - pos2.x, pos1.y - pos2.y, pos1.theta - pos2.theta};
}

const position_t operator*(const double& c, const position_t& pos) {
  return {c * pos.x, c * pos.y, c * pos.theta};
}

const position_t operator*(const position_t& pos1, const position_t& pos2) {
  return {pos1.x * pos2.x, pos1.y * pos2.y, pos1.theta * pos2.theta};
}

const position_t operator/(const position_t& pos, const double& c) {
  return {pos.x / c, pos.y / c, pos.theta / c};
}

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

state_feedback_controller::state_feedback_controller(double cycle) : cycle_(cycle),smith_predictor_(cycle_,zeta_,omega_) {
  // 状態フィードバックゲイン
  // (s+k)^2=s^2+2ks+k^2=0
  // |sI-A|=s^2+(2ζω-k2ω^2)s+ω^2-k1ω^2
  // 2k=2ζω-k2ω^2,k^2=ω^2-k1ω^2
  double k1 = 1 - std::pow(k_, 2) / std::pow(omega_, 2);
  double k2 = 2 * (zeta_ * omega_ - k_) / std::pow(omega_, 2);
  kp_[0]    = k1;
  kp_[1]    = 0.0;
  ki_[0]    = 0.0;
  ki_[1]    = 0.0;
  kd_[0]    = k2;
  kd_[1]    = 0.0;
  for (int i = 0; i < 2; i++) {
    up_[i] = {0.0, 0.0, 0.0};
    ui_[i] = {0.0, 0.0, 0.0};
    ud_[i] = {0.0, 0.0, 0.0};
    u_[i]  = {0.0, 0.0, 0.0};
    e_[i]  = {0.0, 0.0, 0.0};
  }
}

velocity_t state_feedback_controller::update(const model::robot& robot,
                                             const position_t& setpoint) {
  calculate_regulator(robot);
  position_t e_p = setpoint - position_t{estimated_robot_.x(), estimated_robot_.y(),
                                         estimated_robot_.theta()};
  position_t delta_p = convert(e_p, estimated_robot_.theta());

  velocity_t target;
  target.vx    = sliding_mode_[0].control_pos(-delta_p.x);
  target.vy    = sliding_mode_[1].control_pos(-delta_p.y);
  target.omega = clamp(util::wrap_to_pi(delta_p.theta) * 2, -pi<double>(), pi<double>());

  u_[0] = u_[0] + (std::pow(k_, 2) / std::pow(omega_, 2)) * target;

  // 値の更新
  up_[1] = up_[0];
  ui_[1] = ui_[0];
  ud_[1] = ud_[0];
  u_[1]  = u_[0];
  e_[1]  = e_[0];

  return u_[0];
}

velocity_t state_feedback_controller::update(const model::robot& robot,
                                             const velocity_t& setpoint) {
  calculate_regulator(robot);

  velocity_t target = convert(setpoint, estimated_robot_.theta());

  // 加速度，速度制限
  double u_angle     = std::atan2(target.vy, target.vx); // 今回指令速度の方向
  double u_speed     = std::hypot(target.vx, target.vy); // 今回指令速度の大きさ
  double robot_speed = std::hypot(u_[1].vx, u_[1].vy);
  double delta_speed = u_speed - robot_speed; // 速さ偏差(今回指令とロボット速さの差)
  // 速度に応じて加速度を変化(初動でのスリップ防止)
  // 制限加速度計算
  double optimized_accel =
      robot_speed * (max_acceleration_ - min_acceleration_) / reach_speed_ + min_acceleration_;
  optimized_accel = clamp(optimized_accel, min_acceleration_, max_acceleration_);
  // 加速度制限
  if (delta_speed / cycle_ > optimized_accel) {
    if (std::abs(u_speed) > std::abs(robot_speed)) {
      u_speed = robot_speed + (optimized_accel * cycle_); // +方向加速
    }
  } else if (delta_speed / cycle_ < -optimized_accel) {
    if (std::abs(u_speed) > std::abs(robot_speed)) {
      u_speed = robot_speed - (optimized_accel * cycle_); // -方向加速
    }
  }
  // 速度制限
  u_speed = clamp(u_speed, 0.0, max_velocity_);

  // 成分速度再計算
  target.vx    = u_speed * std::cos(u_angle);
  target.vy    = u_speed * std::sin(u_angle);
  target.omega = setpoint.omega;

  u_[0] = u_[0] + (std::pow(k_, 2) / std::pow(omega_, 2)) * target;

  // 値の更新
  up_[1] = up_[0];
  ui_[1] = ui_[0];
  ud_[1] = ud_[0];
  u_[1]  = u_[0];
  e_[1]  = e_[0];

  return u_[0];
}

void state_feedback_controller::calculate_regulator(const model::robot& robot) {
  // 前回制御入力をフィールド基準に座標変換
  velocity_t pre_u;
  double u_direction = std::atan2(u_[1].vy, u_[1].vx);
  pre_u.vx =
      u_[1].vx * std::cos(u_direction) + u_[1].vy * std::cos(u_direction + half_pi<double>());
  pre_u.vy =
      u_[1].vx * std::sin(u_direction) + u_[1].vy * std::sin(u_direction + half_pi<double>());
  pre_u.omega = u_[1].omega;

  // smith_predictorでvisionの遅れ時間の補間
  Eigen::Matrix3d now_state = smith_predictor_.interpolate(robot, pre_u);
  estimated_robot_.set_x(now_state(0, 0));
  estimated_robot_.set_y(now_state(1, 0));
  estimated_robot_.set_theta(now_state(2, 0));
  estimated_robot_.set_vx(now_state(0, 1));
  estimated_robot_.set_vy(now_state(1, 1));
  estimated_robot_.set_omega(now_state(2, 1));

  // ロボット状態を座標変換
  e_[0] = convert(
      velocity_t{estimated_robot_.vx(), estimated_robot_.vy(), estimated_robot_.omega()},
      estimated_robot_.theta());

  // 双一次変換
  // s=(2/T)*(Z-1)/(Z+1)としてPIDcontrollerを離散化
  // C=Kp+Ki/s+Kds
  // 計算用にゲイン再設定
  velocity_t kp = model::command::velocity_t{kp_[0], kp_[0], kp_[1]};
  velocity_t ki = model::command::velocity_t{ki_[0], ki_[0], ki_[1]};
  velocity_t kd = model::command::velocity_t{kd_[0], kd_[0], kd_[1]};
  up_[0]        = kp * e_[0];
  ui_[0]        = cycle_ * ki * (e_[0] + e_[1]) / 2 + ui_[1];
  ud_[0]        = 2 * kd * (e_[0] - e_[1]) / cycle_ - ud_[1];
  u_[0]         = (up_[0] + ui_[0] + ud_[0]);
}

position_t state_feedback_controller::convert(const position_t pos, const double robot_theta) {
  position_t target;
  target.x = pos.x * std::cos(-robot_theta) + pos.y * std::cos(half_pi<double>() - robot_theta);
  target.y = pos.x * std::sin(-robot_theta) + pos.y * std::sin(half_pi<double>() - robot_theta);
  target.theta = pos.theta;

  return target;
}

velocity_t state_feedback_controller::convert(const velocity_t vel, const double robot_theta) {
  velocity_t target;
  target.vx =
      vel.vx * std::cos(-robot_theta) + vel.vy * std::cos(half_pi<double>() - robot_theta);
  target.vy =
      vel.vx * std::sin(-robot_theta) + vel.vy * std::sin(half_pi<double>() - robot_theta);
  target.omega = vel.omega;

  return target;
}

} // controller
} // ai_server
