#include <boost/algorithm/clamp.hpp>
#include <boost/math/constants/constants.hpp>
#include <cmath>

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

state_feedback_controller::state_feedback_controller(double cycle)
    : cycle_(cycle), sliding_mode_{cycle_, cycle_}, smith_predictor_(cycle_, zeta_, omega_) {
  // 状態フィードバックゲイン
  // (s+k)^2=s^2+2ks+k^2=0
  // |sI-A|=s^2+(2ζω-k2ω^2)s+ω^2-k1ω^2
  // 2k=2ζω-k2ω^2,k^2=ω^2-k1ω^2
  double k1 = 1 - std::pow(k_, 2) / std::pow(omega_, 2);
  double k2 = 2 * (zeta_ * omega_ - k_) / std::pow(omega_, 2);
  kp_ << k1, k1, 0.0;
  ki_ << 0.0, 0.0, 0.0;
  kd_ << k2, k2, 0.0;
  for (int i = 0; i < 2; i++) {
    up_[i].Eigen::Vector3d::Zero(3);
    ui_[i].Eigen::Vector3d::Zero(3);
    ud_[i].Eigen::Vector3d::Zero(3);
    u_[i].Eigen::Vector3d::Zero(3);
    e_[i].Eigen::Vector3d::Zero(3);
  }
}

velocity_t state_feedback_controller::update(const model::robot& robot,
                                             const position_t& setpoint) {
  calculate_regulator(robot);
  Eigen::RowVector3d set;
  set << setpoint.x, setpoint.y, setpoint.theta;
  Eigen::Vector3d e_p     = set - estimated_robot_.row(0);
  Eigen::Vector3d delta_p = convert(e_p, estimated_robot_(2));

  Eigen::Vector3d target;
  target(0) = sliding_mode_[0].control_pos(-delta_p(0));
  target(1) = sliding_mode_[1].control_pos(-delta_p(1));
  // 速度が大きいときに角速度が大きくなりすぎないように
  double omega_limit = pi<double>() * std::exp(-std::hypot(target(0), target(1)) / 2000.0);
  target(2)          = clamp(util::wrap_to_pi(delta_p(2)) * 2, -omega_limit, omega_limit);

  u_[0] = u_[0] + (std::pow(k_, 2) / std::pow(omega_, 2)) * target;

  // 値の更新
  up_[1] = up_[0];
  ui_[1] = ui_[0];
  ud_[1] = ud_[0];
  u_[1]  = u_[0];
  e_[1]  = e_[0];

  return velocity_t{u_[0](0), u_[0](1), u_[0](2)};
}

velocity_t state_feedback_controller::update(const model::robot& robot,
                                             const velocity_t& setpoint) {
  calculate_regulator(robot);

  Eigen::Vector3d set;
  set << setpoint.vx, setpoint.vy, setpoint.omega;
  Eigen::Vector3d target = convert(set, estimated_robot_(2));

  // 加速度，速度制限
  double u_angle     = std::atan2(target(1), target(0)); // 今回指令速度の方向
  double u_speed     = std::hypot(target(0), target(1)); // 今回指令速度の大きさ
  double robot_speed = std::hypot(u_[1](0), u_[1](1));
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
  target(0) = u_speed * std::cos(u_angle);
  target(1) = u_speed * std::sin(u_angle);
  // 速度が大きいときに角速度が大きくなりすぎないように
  double omega_limit = pi<double>() * std::exp(-std::hypot(target(0), target(1)) / 2000.0);
  target(2)          = clamp(target(2), -omega_limit, omega_limit);

  u_[0] = u_[0] + (std::pow(k_, 2) / std::pow(omega_, 2)) * target;

  // 値の更新
  up_[1] = up_[0];
  ui_[1] = ui_[0];
  ud_[1] = ud_[0];
  u_[1]  = u_[0];
  e_[1]  = e_[0];

  return velocity_t{u_[0](0), u_[0](1), u_[0](2)};
}

void state_feedback_controller::calculate_regulator(const model::robot& robot) {
  // 前回制御入力をフィールド基準に座標変換
  Eigen::Vector3d pre_u;
  double u_direction = std::atan2(u_[1](1), u_[1](0));
  pre_u(0) =
      u_[1](0) * std::cos(u_direction) + u_[1](1) * std::cos(u_direction + half_pi<double>());
  pre_u(1) =
      u_[1](0) * std::sin(u_direction) + u_[1](1) * std::sin(u_direction + half_pi<double>());
  pre_u(2) = u_[1](2);

  // smith_predictorでvisionの遅れ時間の補間
  estimated_robot_ = smith_predictor_.interpolate(robot, pre_u);

  // ロボット状態を座標変換
  e_[0] = convert(estimated_robot_.row(1), estimated_robot_(0, 2));

  // 双一次変換
  // s=(2/T)*(Z-1)/(Z+1)としてPIDcontrollerを離散化
  // C=Kp+Ki/s+Kds
  for (int i = 0; i < 3; i++) {
    up_[0](i) = kp_(i) * e_[0](i);
    ui_[0](i) = cycle_ * ki_(i) * (e_[0](i) + e_[1](i)) / 2.0 + ui_[1](i);
    ud_[0](i) = 2.0 * kd_(i) * (e_[0](i) - e_[1](i)) / cycle_ - ud_[1](i);
  }
  u_[0] = (up_[0] + ui_[0] + ud_[0]);
}

Eigen::Vector3d state_feedback_controller::convert(const Eigen::Vector3d raw,
                                                   const double robot_theta) {
  Eigen::Vector3d target;
  target(0) =
      raw(0) * std::cos(-robot_theta) + raw(1) * std::cos(half_pi<double>() - robot_theta);
  target(1) =
      raw(0) * std::sin(-robot_theta) + raw(1) * std::sin(half_pi<double>() - robot_theta);
  target(2) = raw(2);

  return target;
}

} // controller
} // ai_server
