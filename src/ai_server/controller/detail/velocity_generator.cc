#include <algorithm>
#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <cmath>

#include "velocity_generator.h"

namespace ai_server {
namespace controller {
namespace detail {

const double velocity_generator::v_max_       = 8000.0;
const double velocity_generator::a_max_       = 8000.0;
const double velocity_generator::a_min_       = 5000.0;
const double velocity_generator::reach_speed_ = 1000.0;
const double velocity_generator::kp_          = 1.0;

velocity_generator::velocity_generator(double cycle) : cycle_(cycle) {
  v_target_ = 0.0;
}

double velocity_generator::control_pos(const double delta_p, const bool stable) {
  if (std::abs(delta_p) > 1000.0) {
    if (delta_p > 0) {
      if (2 * std::pow(v_target_, 2) / (a_max_ * 2) > std::abs(delta_p)) {
        v_target_ -= a_max_ * cycle_;
        v_target_ = std::clamp(v_target_, 0.0, v_max_);
      } else {
        v_target_ += a_min_ * cycle_;
      }
    } else {
      if (2 * std::pow(v_target_, 2) / (a_max_ * 2) > std::abs(delta_p)) {
        v_target_ += a_max_ * cycle_;
        v_target_ = std::clamp(v_target_, -v_max_, 0.0);
      } else {
        v_target_ -= a_min_ * cycle_;
      }
    }
    v_target_ = std::clamp(v_target_, -v_max_, v_max_);
  } else {
    double k;
    if (stable) {
      k = kp_;
    } else {
      k = kp_ * 10.0 * std::exp(-std::abs(v_target_) / 1000.0);
    }
    double a_required = std::abs(delta_p) * std::pow(k, 2);
    double state;

    // 必要な収束加速度が最大加速度を上回り,bangbang制御が必要か
    if (a_required < a_max_) {
      // 普通のsliding_mode用state
      state = k * -delta_p + v_target_;
    } else {
      // bangbang制御をするための特別なstate関数
      // 極度に非線形なため正か負かでstate切り替え
      int sgn   = boost::math::sign(-delta_p);
      double pm = sgn * a_max_ / std::pow(k, 2);
      double vm = -sgn * a_max_ / k;
      state     = sgn * std::sqrt(sgn * (-delta_p - pm)) + (v_target_ - vm);
    }

    // sliding_mode
    if (std::abs(state) < a_max_ * cycle_) {
      v_target_ = -k * -delta_p; // stateが0になるように速度を保つ
    } else if (state < 0) {
      v_target_ += a_min_ * cycle_; // stateが-なら加速
    } else {
      v_target_ -= a_min_ * cycle_; // stateが+なら減速
    }
  }
  return v_target_;
}

double velocity_generator::control_vel(const double target, const bool stable) {
  double state = v_target_ - target;
  // 制限加速度計算
  // 速度によって加速度が変化,初動でスリップしないように
  double optimized_accel = v_target_ * (a_max_ - a_min_) / reach_speed_ + a_min_;
  optimized_accel        = boost::algorithm::clamp(optimized_accel, a_min_, a_max_);

  if (stable) {
    optimized_accel = a_min_ / 2.0;
  } else if (std::abs(v_target_) > std::abs(target) && v_target_ * target > 0) {
    optimized_accel = a_max_;
  } else {
    optimized_accel = a_min_;
  }
  if (std::abs(state) < optimized_accel * cycle_) {
    v_target_ = target;
  } else if (state < 0) {
    v_target_ += optimized_accel * cycle_; // stateが-なら加速
  } else {
    v_target_ -= optimized_accel * cycle_; // stateが+なら減速
  }
  // nop
  if (target == 0.0) {
    v_target_ = 0.0;
  }
  return v_target_;
}

} // namespace detail
} // namespace controller
} // namespace ai_server
