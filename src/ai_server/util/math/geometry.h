#ifndef AI_SERVER_UTIL_GEOMETRY_H
#define AI_SERVER_UTIL_GEOMETRY_H

#include <boost/math/constants/constants.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>

namespace ai_server {
namespace util {
/// @brief     ある直線の始点と終点が与えられたとき,終点から左右に任意の長さ分ずらした2点を返す
/// @param begin ある直線についての始点
/// @param end ある直線についての終点
/// @param shift 終点からずらしたい長さ
/// @return
/// std::tuple<Eigen::Vector2d,Eigen::Vector2d>{終点から右にずらした点,終点から左にずらした点}
template <class T>
auto calc_vertex(Eigen::Vector2d begin, Eigen::Vector2d end, T shift) {
  using boost::math::constants::pi;

  const Eigen::Rotation2Dd rotate(pi<double>() / 2.0);
  //正規化した物をpi/2回転させる
  const auto normalize = rotate * ((begin - end).normalized());
  //+-shiftずらして終点に足す
  std::tuple<Eigen::Vector2d, Eigen::Vector2d> t =
      std::make_tuple((end + normalize * shift), (end + normalize * -shift));
  return t;
}
}
}
#endif
