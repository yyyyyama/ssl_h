#include <cmath>

#include "sliding_mode_controller.h"
namespace ai_server {
namespace controller {

const double sliding_mode_controller::a_max_ = 2000.0;
const double sliding_mode_controller::kp_    = 1.0;

sliding_mode_controller::sliding_mode_controller(double cycle) : cycle_(cycle) {
  v_target_ = 0.0;
}

double sliding_mode_controller::control(const double delta_p) {
  double a_required = std::abs(delta_p) * std::pow(kp_, 2);
  double state;

  // 必要な収束加速度が最大加速度を上回り,bangbang制御が必要か
  if (a_required < a_max_) {
    // 普通のSlidingMode
    state = kp_ * delta_p + v_target_;
  } else {
    // bangbang制御をするための特別なstate関数
    // 極度に非線形なため正か負かで場合分け。
    if (delta_p > 0) {
      double pm = a_max_ / std::pow(kp_, 2);
      double vm = -a_max_ / kp_;
      state     = std::pow(delta_p - pm, 1 / 2.0) + (v_target_ - vm);
    } else {
      double pm = -a_max_ / std::pow(kp_, 2);
      double vm = a_max_ / kp_;
      state     = -std::pow(-(delta_p - pm), 1 / 2.0) + (v_target_ - vm);
    }
  }

  if (state < a_max_ * cycle_ && state > -a_max_ * cycle_) {
    v_target_ = -kp_ * delta_p; // stateが0になるように速度を保つ
  } else if (state < 0) {       // stateが-なら加速
    v_target_ += a_max_ * cycle_;
  } else { // stateが+なら減速
    v_target_ -= a_max_ * cycle_;
  }
  return v_target_;
}

} // controller
} // ai_server
