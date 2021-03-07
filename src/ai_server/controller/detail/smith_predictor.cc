#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>

#include "smith_predictor.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

namespace ai_server {
namespace controller {
namespace detail {

// 遅延は 40-60 [ms]
// 参考: https://ssl.robocup.org/wp-content/uploads/2020/03/2020_ETDP_ZJUNlict.pdf
const double smith_predictor::delay_ = 0.05;

smith_predictor::smith_predictor(double cycle)
    : cycle_(cycle), u_(static_cast<std::size_t>(delay_ / cycle), Eigen::Vector3d::Zero()) {}

Eigen::Matrix3d smith_predictor::interpolate(const model::robot& robot,
                                             const Eigen::Vector3d& u) {
  // 回転による座標変化を考慮
  const Eigen::Vector3d corrected_u =
      Eigen::AngleAxisd(cycle_ * u.z(), Eigen::Vector3d::UnitZ()) * u;

  const Eigen::Vector3d a = corrected_u - u_.back();

  Eigen::Vector3d p =
      std::accumulate(u_.begin(), u_.end(), util::math::position3d(robot),
                      [this](const auto& p, const auto& v) { return p + cycle_ * v; }) +
      std::fmod(delay_, cycle_) * corrected_u;
  // 正規化
  p.z() = util::math::wrap_to_pi(p.z());

  // 更新
  u_.push_back(corrected_u);
  u_.pop_front();

  return (Eigen::Matrix3d() << p, corrected_u, a).finished();
}
} // namespace detail
} // namespace controller
} // namespace ai_server
