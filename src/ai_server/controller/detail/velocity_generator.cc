#include <algorithm>
#include <boost/math/special_functions/sign.hpp>
#include <cmath>

#include "velocity_generator.h"

namespace ai_server {
namespace controller {
namespace detail {

const double velocity_generator::v_max_       = 3000.0;
const double velocity_generator::a_max_       = 5000.0;
const double velocity_generator::a_min_       = 5000.0;
const double velocity_generator::reach_speed_ = 1000.0;
const double velocity_generator::kp_          = 3.5;

velocity_generator::velocity_generator(const double cycle) : cycle_(cycle) {}

double velocity_generator::control_pos(const double pre_vel, const double delta_p,
                                       const bool stable) const {
  double v_target              = pre_vel;
  const double k               = stable ? 0.5 * kp_ : kp_;
  const double optimized_accel = a_max_;

  // 普通のsliding_mode用state
  const double state = k * -delta_p + v_target;

  // sliding_mode
  if (std::abs(state) < a_max_ * cycle_) {
    v_target = -k * -delta_p; // stateが0になるように速度を保つ
  } else if (state < 0) {
    v_target += optimized_accel * cycle_; // stateが-なら加速
  } else {
    v_target -= optimized_accel * cycle_; // stateが+なら減速
  }
  // velocity limit
  const double v_max = std::min(v_max_, 2.0 * optimized_accel / k);
  v_target           = std::clamp(v_target, -v_max, v_max);
  return v_target;
}

double velocity_generator::control_vel(const double pre_vel, const double target,
                                       const bool stable) const {
  double v_target = pre_vel;
  // 制限加速度計算
  // 速度によって加速度が変化,初動でスリップしないように
  double optimized_accel = v_target * (a_max_ - a_min_) / reach_speed_ + a_min_;
  optimized_accel        = std::clamp(optimized_accel, a_min_, a_max_);

  if (stable) {
    optimized_accel = a_min_ / 2.0;
  } else if (std::abs(v_target) > std::abs(target) && v_target * target > 0) {
    optimized_accel = a_max_;
  } else {
    optimized_accel = a_min_;
  }

  v_target = std::clamp(target, v_target - optimized_accel * cycle_,
                        v_target + optimized_accel * cycle_);
  v_target = std::clamp(v_target, -v_max_, v_max_);
  return v_target;
}

double velocity_generator::a_max() const {
  return a_max_;
}

} // namespace detail
} // namespace controller
} // namespace ai_server
