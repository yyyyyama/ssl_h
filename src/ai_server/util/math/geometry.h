#ifndef AI_SERVER_UTIL_GEOMETRY_H
#define AI_SERVER_UTIL_GEOMETRY_H

#include <boost/math/constants/constants.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include "ai_server/util/math.h"

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
std::tuple<Eigen::Vector2d, Eigen::Vector2d> calc_isosceles_vertexes(
    const Eigen::Vector2d& apex, const Eigen::Vector2d& middle_base, T shift) {
  const Eigen::Vector2d move{apex};

  //計算の為に中心にずらした場合の座標
  Eigen::Vector2d after_apex{apex - move};

  Eigen::Vector2d after_middle_base{middle_base - move};

  // x軸から角度
  const auto alpha = util::wrap_to_2pi(std::atan2(after_middle_base.y() - after_apex.y(),
                                                  after_middle_base.x() - after_apex.x()));
  //回転行列
  const Eigen::Rotation2D<T> rotate(alpha);

  after_middle_base.x() = (after_middle_base - after_apex).norm();
  after_middle_base.y() = 0.0;

  //移動した先での仮の座標
  const Eigen::Vector2d tmp1(after_middle_base.x(), shift);
  const Eigen::Vector2d tmp2(tmp1.x(), tmp1.y() * (-1));

  //回転した後の正しい座標
  return std::make_tuple((rotate * tmp1) + move, (rotate * tmp2) + move);
}
}
}
}
#endif
