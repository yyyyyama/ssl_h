#include <boost/math/special_functions/sign.hpp>
#include <cmath>

#include "sliding_mode_controller.h"

namespace ai_server {
namespace controller {
namespace detail {

const double sliding_mode_controller::a_max_ = 4000.0;
const double sliding_mode_controller::kp_    = 1.0;

sliding_mode_controller::sliding_mode_controller(double cycle) : cycle_(cycle) {
  v_target_ = 0.0;
}

double sliding_mode_controller::control_pos(const double delta_p) {
  double a_required = std::abs(delta_p) * std::pow(kp_, 2);
  double state;

  // 必要な収束加速度が最大加速度を上回り,bangbang制御が必要か
  if (a_required < a_max_) {
    // 普通のSlidingMode
    state = kp_ * delta_p + v_target_;
  } else {
    // bangbang制御をするための特別なstate関数
    // 極度に非線形なため正か負かでstate切り替え
    int sgn   = boost::math::sign(delta_p);
    double pm = sgn * a_max_ / std::pow(kp_, 2);
    double vm = -sgn * a_max_ / kp_;
    state     = sgn * std::sqrt(sgn * (delta_p - pm)) + (v_target_ - vm);
  }

  if (std::abs(state) < a_max_ * cycle_) {
    v_target_ = -kp_ * delta_p; // stateが0になるように速度を保つ
  } else if (state < 0) {
    v_target_ += a_max_ * cycle_; // stateが-なら加速
  } else {
    v_target_ -= a_max_ * cycle_; // stateが+なら減速
  }
  return v_target_;
}

double sliding_mode_controller::control_vel(const double delta_v) {
  double state = delta_v;
  if (std::abs(state) < a_max_ * cycle_) {
    v_target_ += kp_ * delta_v; // stateが0になるように速度を保つ
  } else if (state < 0) {
    v_target_ += a_max_ * cycle_; // stateが-なら加速
  } else {
    v_target_ -= a_max_ * cycle_; // stateが+なら減速
  }
  return v_target_;
}

} // detail
} // controller
} // ai_server
