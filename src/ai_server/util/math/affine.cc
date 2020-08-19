#include "ai_server/util/math/angle.h"
#include "affine.h"
#include "to_vector.h"

namespace ai_server {
namespace util {
namespace math {

model::ball transform(const Eigen::Affine3d& matrix, const model::ball& ball) {
  // 回転方向は必要ないので
  Eigen::Vector3d p{.0, .0, .0};
  p.head<2>() = util::math::position(ball);

  // matrixで座標の変換をする
  const auto r = matrix * p;

  auto result = ball;
  result.set_x(r.x());
  result.set_y(r.y());
  return result;
}

model::robot transform(const Eigen::Affine3d& matrix, const model::robot& robot) {
  // matrixで座標の変換をする
  const auto r = matrix * util::math::position3d(robot);

  auto result = robot;
  result.set_x(r.x());
  result.set_y(r.y());
  result.set_theta(util::math::wrap_to_2pi(r.z()));
  return result;
}

Eigen::Vector2d transform(const Eigen::Affine3d& matrix, const Eigen::Vector2d& v) {
  // 回転方向は必要ないので
  Eigen::Vector3d p{.0, .0, .0};
  p.head<2>() = v;

  return (matrix * p).head<2>();
}
} // namespace math
} // namespace util
} // namespace ai_server
