#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "ai_server/util/math.h"
#include "state_feedback_controller.h"

namespace ai_server {
namespace controller {

using boost::math::constants::half_pi;
using boost::math::constants::pi;

const double state_feedback_controller::k_     = 49.17;
const double state_feedback_controller::zeta_  = 1.0;
const double state_feedback_controller::omega_ = 49.17;
const double state_feedback_controller::v_max_ = 5000.0;

state_feedback_controller::state_feedback_controller(double cycle, const model::world& world)
    : base(v_max_),
      cycle_(cycle),
      world_(world),
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

void state_feedback_controller::set_velocity_limit(const double limit) {
  base::set_velocity_limit(std::min(limit, v_max_));
}

velocity_t state_feedback_controller::update(const model::robot& robot,
                                             const position_t& setpoint) {
  calculate_regulator(robot);
  Eigen::Vector3d set     = {setpoint.x, setpoint.y, setpoint.theta};
  Eigen::Vector3d e_p     = set - estimated_robot_.col(0);
  Eigen::Vector3d delta_p = convert(e_p, estimated_robot_(2, 0));
  double target_angle     = std::atan2(delta_p.y(), delta_p.x());

  Eigen::Vector3d target;
  target.x() = velocity_generator_[0].control_pos(-delta_p.x());
  target.y() = velocity_generator_[1].control_pos(-delta_p.y());
  target.z() = util::math::wrap_to_pi(delta_p.z()) * 2.0;

  calculate_output(target, target_angle);

  return velocity_t{u_[0].x(), u_[0].y(), u_[0].z()};
}

velocity_t state_feedback_controller::update(const model::robot& robot,
                                             const velocity_t& setpoint) {
  calculate_regulator(robot);

  Eigen::Vector3d set    = {setpoint.vx, setpoint.vy, setpoint.omega};
  Eigen::Vector3d target = convert(set, estimated_robot_(2, 0));
  double target_angle    = std::atan2(setpoint.vy, setpoint.vx);

  target.x() = velocity_generator_[0].control_vel(target.x());
  target.y() = velocity_generator_[1].control_vel(target.y());
  target.z() = set.z();

  calculate_output(target, target_angle);

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

// 出力計算及び後処理
void state_feedback_controller::calculate_output(Eigen::Vector3d target, double target_angle) {
  // 速度が大きいときに角速度が大きくなりすぎないように
  double omega_limit = pi<double>() * std::exp(-std::hypot(target.x(), target.y()) / 2000.0);
  target.z() = std::clamp(util::math::wrap_to_pi(target.z()) * 2, -omega_limit, omega_limit);

  // スピンしてる(角速度が大きすぎる)ときは速度落とす
  if (estimated_robot_(2, 1) > 10) {
    target = Eigen::Vector3d::Zero();
  }

  // ロボット入力計算
  u_[0] = u_[0] + (std::pow(k_, 2) / std::pow(omega_, 2)) * target;
  // nanが入ったら前回入力を今回値とする
  if (std::isnan(u_[0].x()) || std::isnan(u_[0].y()) || std::isnan(u_[0].z())) {
    u_[0] = u_[1];
  }

  double vx_max         = velocity_limit_;
  double vx_min         = velocity_limit_;
  double vy_max         = velocity_limit_;
  double vy_min         = velocity_limit_;
  double margin_outside = 500.0;
  double margin_inside  = 1500.0;
  double width          = margin_outside + margin_inside;
  // フィールドに対して外に出そうなやつは速度制限を強める
  if (estimated_robot_(0, 0) > world_.field().x_max() - margin_inside) {
    vx_max *= (world_.field().x_max() + margin_outside - estimated_robot_(0, 0)) / width;
    if (vx_max < 0) {
      vx_max = 0.0;
    }
  }
  if (estimated_robot_(0, 0) < world_.field().x_min() + margin_inside) {
    vx_min *= (world_.field().x_min() - margin_outside - estimated_robot_(0, 0)) / width;
    if (vx_min > 0) {
      vx_min = 0.0;
    }
  }
  if (estimated_robot_(1, 0) > world_.field().y_max() - margin_inside) {
    vy_max *= (world_.field().y_max() + margin_outside - estimated_robot_(1, 0)) / width;
    if (vy_max < 0) {
      vy_max = 0.0;
    }
  }
  if (estimated_robot_(1, 0) < world_.field().y_min() + margin_inside) {
    vy_min *= (world_.field().y_min() - margin_outside - estimated_robot_(1, 0)) / width;
    if (vy_min > 0) {
      vy_min = 0.0;
    }
  }
  // 各方向の速度制限線が作る四角形がロボットの速度制限
  // 2直線の交点を求めることでロボットのxyに対しての速度制限計算
  double posi_x;
  double nega_x;
  double posi_y;
  double nega_y;
  if (estimated_robot_(2, 0) < -half_pi<double>()) {
    posi_x = find_cross_point(vx_min, vy_max, estimated_robot_(2, 0) + half_pi<double>());
    posi_y = find_cross_point(vx_min, vy_min, estimated_robot_(2, 0));
    nega_x = find_cross_point(vx_max, vy_min, estimated_robot_(2, 0) + half_pi<double>());
    nega_y = find_cross_point(vx_max, vy_max, estimated_robot_(2, 0));
  } else if (estimated_robot_(2, 0) < 0) {
    posi_x = find_cross_point(vx_min, vy_min, estimated_robot_(2, 0) - half_pi<double>());
    posi_y = find_cross_point(vx_max, vy_min, estimated_robot_(2, 0));
    nega_x = find_cross_point(vx_max, vy_max, estimated_robot_(2, 0) + half_pi<double>());
    nega_y = find_cross_point(vx_min, vy_max, estimated_robot_(2, 0));
  } else if (estimated_robot_(2, 0) < half_pi<double>()) {
    posi_x = find_cross_point(vx_max, vy_min, estimated_robot_(2, 0) - half_pi<double>());
    posi_y = find_cross_point(vx_max, vy_max, estimated_robot_(2, 0));
    nega_x = find_cross_point(vx_min, vy_max, estimated_robot_(2, 0) + half_pi<double>());
    nega_y = find_cross_point(vx_min, vy_min, estimated_robot_(2, 0));
  } else {
    posi_x = find_cross_point(vx_max, vy_max, estimated_robot_(2, 0) - half_pi<double>());
    posi_y = find_cross_point(vx_min, vy_max, estimated_robot_(2, 0));
    nega_x = find_cross_point(vx_min, vy_min, estimated_robot_(2, 0) - half_pi<double>());
    nega_y = find_cross_point(vx_max, vy_min, estimated_robot_(2, 0));
  }
  // 速度制限
  u_[0].x() = std::clamp(u_[0].x(), -nega_x, posi_x);
  u_[0].y() = std::clamp(u_[0].y(), -nega_y, posi_y);

  // 値の更新
  up_[1] = up_[0];
  ui_[1] = ui_[0];
  ud_[1] = ud_[0];
  u_[1]  = u_[0];
  e_[1]  = e_[0];
}

double state_feedback_controller::find_cross_point(const double x_1, const double y_2,
                                                   const double angle) {
  // 2点から直線の式を求め,交点を求める
  // (y_1,x_2はゼロ)
  double x;
  double y;
  if (std::abs(angle) == half_pi<double>()) {
    return y_2;
  }
  if (x_1 == 0.0 || y_2 == 0.0) {
    return 0.0;
  }
  x = x_1 * y_2 / (x_1 * std::tan(angle) + y_2);
  y = -x * y_2 / x_1 + x_1 * y_2 / x_1;

  // 原点から交点までの距離を返す
  return std::hypot(x, y);
}

} // namespace controller
} // namespace ai_server
