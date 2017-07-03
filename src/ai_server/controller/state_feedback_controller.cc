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

const double state_feedback_controller::k_     = 49.17;
const double state_feedback_controller::zeta_  = 1.0;
const double state_feedback_controller::omega_ = 49.17;

state_feedback_controller::state_feedback_controller(double cycle)
    : cycle_(cycle),
      velocity_generator_{cycle_, cycle_},
      smith_predictor_(cycle_, zeta_, omega_) {
  // 状態フィードバックゲイン
  // (s+k)^2=s^2+2ks+k^2=0
  // |sI-A|=s^2+(2ζω-k2ω^2)s+ω^2-k1ω^2
  // 2k=2ζω-k2ω^2,k^2=ω^2-k1ω^2
  double k1 = 1 - std::pow(k_, 2) / std::pow(omega_, 2);
  double k2 = 2 * (zeta_ * omega_ - k_) / std::pow(omega_, 2);
  kp_       = {k1, k1, 0.0};
  ki_       = {0.0, 0.0, 0.0};
  kd_       = {k2, k2, 0.0};
  for (int i = 0; i < 2; i++) {
    up_[i] = Eigen::Vector3d::Zero();
    ui_[i] = Eigen::Vector3d::Zero();
    ud_[i] = Eigen::Vector3d::Zero();
    u_[i]  = Eigen::Vector3d::Zero();
    e_[i]  = Eigen::Vector3d::Zero();
  }
}

velocity_t state_feedback_controller::update(const model::robot& robot,
                                             const position_t& setpoint) {
  calculate_regulator(robot);
  Eigen::Vector3d set     = {setpoint.x, setpoint.y, setpoint.theta};
  Eigen::Vector3d e_p     = set - estimated_robot_.col(0);
  Eigen::Vector3d delta_p = convert(e_p, estimated_robot_(2, 0));

  Eigen::Vector3d target;
  target.x() = velocity_generator_[0].control_pos(-delta_p.x());
  target.y() = velocity_generator_[1].control_pos(-delta_p.y());
  // 速度が大きいときに角速度が大きくなりすぎないように
  double omega_limit = pi<double>() * std::exp(-std::hypot(target.x(), target.y()) / 2000.0);
  target.z()         = clamp(util::wrap_to_pi(delta_p.z()) * 2, -omega_limit, omega_limit);

  u_[0] = u_[0] + (std::pow(k_, 2) / std::pow(omega_, 2)) * target;
  // nanが入ったら前回入力を今回値とする
  if (std::isnan(u_[0].x()) || std::isnan(u_[0].y()) || std::isnan(u_[0].z())) {
    u_[0] = u_[1];
  }

  // 値の更新
  up_[1] = up_[0];
  ui_[1] = ui_[0];
  ud_[1] = ud_[0];
  u_[1]  = u_[0];
  e_[1]  = e_[0];

  return velocity_t{u_[0].x(), u_[0].y(), u_[0].z()};
}

velocity_t state_feedback_controller::update(const model::robot& robot,
                                             const velocity_t& setpoint) {
  calculate_regulator(robot);

  Eigen::Vector3d set    = {setpoint.vx, setpoint.vy, setpoint.omega};
  Eigen::Vector3d target = convert(set, estimated_robot_(2, 0));

  target.x() = velocity_generator_[0].control_vel(target.x());
  target.y() = velocity_generator_[1].control_vel(target.y());
  // 速度が大きいときに角速度が大きくなりすぎないように
  double omega_limit = pi<double>() * std::exp(-std::hypot(target.x(), target.y()) / 2000.0);
  target.z()         = clamp(set.z(), -omega_limit, omega_limit);

  u_[0] = u_[0] + (std::pow(k_, 2) / std::pow(omega_, 2)) * target;
  // nanが入ったら前回入力を今回値とする
  if (std::isnan(u_[0].x()) || std::isnan(u_[0].y()) || std::isnan(u_[0].z())) {
    u_[0] = u_[1];
  }

  // 値の更新
  up_[1] = up_[0];
  ui_[1] = ui_[0];
  ud_[1] = ud_[0];
  u_[1]  = u_[0];
  e_[1]  = e_[0];

  return velocity_t{u_[0].x(), u_[0].y(), u_[0].z()};
}

void state_feedback_controller::calculate_regulator(const model::robot& robot) {
  // 前回制御入力をフィールド基準に座標変換
  double u_direction    = std::atan2(u_[1].y(), u_[1].x());
  Eigen::Vector3d pre_u = Eigen::AngleAxisd(u_direction, Eigen::Vector3d::UnitZ()) * u_[1];

  // smith_predictorでvisionの遅れ時間の補間
  estimated_robot_ = smith_predictor_.interpolate(robot, pre_u);

  // ロボット速度を座標変換
  e_[0] = convert(estimated_robot_.col(1), estimated_robot_(2, 0));

  // 双一次変換
  // s=(2/T)*(Z-1)/(Z+1)としてPIDcontrollerを離散化
  // C=Kp+Ki/s+Kds
  up_[0] = kp_.array() * e_[0].array();
  ui_[0] = cycle_ * ki_.array() * (e_[0].array() + e_[1].array()) / 2.0 + ui_[1].array();
  ud_[0] = 2.0 * kd_.array() * (e_[0].array() - e_[1].array() / cycle_ - ud_[1].array());

  u_[0] = up_[0] + ui_[0] + ud_[0];
}

Eigen::Vector3d state_feedback_controller::convert(const Eigen::Vector3d& raw,
                                                   const double robot_theta) {
  Eigen::Vector3d target = Eigen::AngleAxisd(-robot_theta, Eigen::Vector3d::UnitZ()) * raw;
  return target;
}

} // controller
} // ai_server
