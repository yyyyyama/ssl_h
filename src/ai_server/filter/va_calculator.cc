#include <cmath>
#include <limits>
#include <boost/math/constants/constants.hpp>

#include "ai_server/model/robot.h"
#include "ai_server/util/math/angle.h"
#include "va_calculator.h"

namespace ai_server {
namespace filter {

template <>
std::optional<model::robot> va_calculator<model::robot>::va_calculator::update(
    std::optional<model::robot> value, std::chrono::system_clock::time_point time) {
  // 対象がロストしたらロストさせる
  if (!value.has_value()) {
    // 再び見えるようになった時に変な値が計算されないように prev_time_ を初期化する
    prev_time_ = time_point_type::min();
    return std::nullopt;
  }

  auto result = std::move(*value);

  if (prev_time_ == time_point_type::min()) {
    // 初めてupdateが呼ばれたときは速度加速度を計算しない
    result.set_vx(0);
    result.set_vy(0);
    result.set_omega(0);
    result.set_ax(0);
    result.set_ay(0);
    result.set_alpha(0);
  } else {
    namespace bmc = boost::math::double_constants;

    // 前回呼ばれたときからの経過時間
    const auto dt = std::chrono::duration<double>{time - prev_time_}.count();
    // 非常に短い間隔でupdateが呼び出されたら直前の値を返す
    // (ゼロ除算の原因になるので)
    if (std::abs(dt) < std::numeric_limits<double>::epsilon()) return prev_state_;

    // 速度の計算
    result.set_vx((result.x() - prev_state_.x()) / dt);
    result.set_vy((result.y() - prev_state_.y()) / dt);

    // 角速度の計算
    // 境界で大きな値になるのを防ぐために, 偏差がpi以下かそうでないかで処理を変える
    // https://github.com/kiksworks/ai-server/pull/63#pullrequestreview-29453425
    const auto dtheta =
        util::math::wrap_to_2pi(result.theta()) - util::math::wrap_to_2pi(prev_state_.theta());
    if (std::abs(dtheta) < bmc::pi) {
      result.set_omega(dtheta / dt);
    } else {
      const auto dtheta2 =
          util::math::wrap_to_pi(result.theta()) - util::math::wrap_to_pi(prev_state_.theta());
      result.set_omega(dtheta2 / dt);
    }

    // 加速度の計算
    result.set_ax((result.vx() - prev_state_.vx()) / dt);
    result.set_ay((result.vy() - prev_state_.vy()) / dt);
    result.set_alpha((result.omega() - prev_state_.omega()) / dt);
  }

  prev_state_ = result;
  prev_time_  = time;
  return result;
}

} // namespace filter
} // namespace ai_server
