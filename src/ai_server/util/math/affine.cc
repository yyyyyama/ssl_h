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
  const Eigen::Vector3d rp = matrix * p;

  auto result = ball;
  result.set_x(rp.x());
  result.set_y(rp.y());
  return result;
}

model::robot transform(const Eigen::Affine3d& matrix, const model::robot& robot) {
  // matrixで座標の変換をする
  const Eigen::Vector3d rp = matrix * util::math::position3d(robot);

  auto result = robot;
  result.set_x(rp.x());
  result.set_y(rp.y());
  result.set_theta(util::math::wrap_to_2pi(rp.z()));
  return result;
}

model::ball transform_all(const Eigen::Affine3d& matrix, const model::ball& ball) {
  // 回転方向は必要ないので
  Eigen::Vector3d p{.0, .0, .0};
  p.head<2>() = util::math::position(ball);
  Eigen::Vector3d v{.0, .0, .0};
  v.head<2>() = util::math::velocity(ball);
  Eigen::Vector3d a{.0, .0, .0};
  a.head<2>() = util::math::acceleration(ball);

  // matrixで座標の変換をする
  const Eigen::Vector3d rp = matrix * p;
  const Eigen::Vector3d rv = matrix.rotation() * v;
  const Eigen::Vector3d ra = matrix.rotation() * a;

  auto result = ball;
  result.set_x(rp.x());
  result.set_y(rp.y());
  result.set_vx(rv.x());
  result.set_vy(rv.y());
  result.set_ax(ra.x());
  result.set_ay(ra.y());
  return result;
}

model::robot transform_all(const Eigen::Affine3d& matrix, const model::robot& robot) {
  // matrixで座標の変換をする
  const Eigen::Vector3d rp = matrix * util::math::position3d(robot);
  const Eigen::Vector3d rv = matrix.rotation() * util::math::velocity3d(robot);
  const Eigen::Vector3d ra = matrix.rotation() * util::math::acceleration3d(robot);

  auto result = robot;
  result.set_x(rp.x());
  result.set_y(rp.y());
  result.set_theta(util::math::wrap_to_2pi(rp.z()));
  result.set_vx(rv.x());
  result.set_vy(rv.y());
  result.set_ax(ra.x());
  result.set_ay(ra.y());
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
