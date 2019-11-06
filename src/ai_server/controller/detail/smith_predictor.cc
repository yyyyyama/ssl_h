#include <cmath>
#include "smith_predictor.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

namespace ai_server {
namespace controller {
namespace detail {

smith_predictor::smith_predictor(double cycle, double zeta, double omega)
    : cycle_(cycle), zeta_(zeta), omega_(omega), u_(7, Eigen::Vector3d::Zero()) {}

Eigen::Matrix3d smith_predictor::interpolate(const model::robot& robot,
                                             const Eigen::Vector3d& u) {
  // 更新
  u_.push_back(u);
  u_.pop_front();

  // 受け取ったロボットの状態(無駄時間分の遅れ含む)
  // 計算しやすいように,ベクトルに変換
  auto now_p = util::math::position3d(robot);
  auto now_v = util::math::velocity3d(robot);
  auto now_a = util::math::acceleration3d(robot);

  // 1ループ毎に当時の制御入力によって1フレーム分の補間をする
  for (const auto& u_n : u_) {
    const auto pre_v = now_v;

    // p=p+v*dt
    now_p += cycle_ * pre_v;
    // v=v+a*dt
    now_v += cycle_ * now_a;
    // a=a+(-omega^2*v-2*zeta*omega*a+omega^2*u)*dt
    now_a += cycle_ * (-std::pow(omega_, 2) * pre_v - 2 * zeta_ * omega_ * now_a +
                       std::pow(omega_, 2) * u_n);
  }

  // 返すために変換
  Eigen::Matrix3d now_state;
  now_state.col(0) = now_p;
  now_state.col(1) = now_v;
  now_state.col(2) = now_a;
  // 角度情報が荒ぶらないように正規化
  now_state(2, 0) = util::math::wrap_to_pi(now_state(2, 0));

  return now_state;
}
} // namespace detail
} // namespace controller
} // namespace ai_server
