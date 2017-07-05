#ifndef AI_SERVER_UTIL_GEOMETRY_H
#define AI_SERVER_UTIL_GEOMETRY_H

#include <boost/math/constants/constants.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>

namespace ai_server {
namespace util {
namespace math {
/// @brief     ある直線の始点と終点が与えられたとき,終点から左右に任意の長さ分ずらした2点を返す
/// @param apex ある直線についての始点
/// @param middle_base ある直線についての終点
/// @param shift 終点からずらしたい長さ
/// @return
/// std::tuple<Eigen::Vector2d,Eigen::Vector2d>{終点から右にずらした点,終点から左にずらした点}
template <class T, std::enable_if_t<std::is_floating_point<T>::value, std::nullptr_t> = nullptr>
auto calc_isosceles_vertexes(const Eigen::Matrix<T, 2, 1>& apex,
                             const Eigen::Matrix<T, 2, 1>& middle_base, T shift) {
  using boost::math::constants::half_pi;

  const Eigen::Rotation2Dd rotate(half_pi<double>());
  //正規化した物をpi/2回転させる
  const auto normalize = rotate * ((apex - middle_base).normalized());

  //+-shiftずらして終点に足す
  return std::make_tuple((middle_base + normalize * shift), (middle_base + normalize * -shift));
}
}
}
}
#endif
