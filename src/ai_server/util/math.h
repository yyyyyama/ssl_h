#ifndef AI_SERVER_UTIL_MATH_H
#define AI_SERVER_UTIL_MATH_H

#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <tuple>

namespace ai_server {
namespace util {
template <class T>
T wrap_to_2pi(T r) {
  using boost::math::constants::two_pi;

  auto wrapped = std::fmod(r, two_pi<T>());

  if (r < 0) {
    wrapped += two_pi<T>();
  }
  return wrapped;
}

/// @brief	-pi<r<=piに正規化
template <class T>
T wrap_to_pi(T r) {
  using boost::math::constants::two_pi;
  using boost::math::constants::pi;

  auto wrapped = std::fmod(r, two_pi<T>());

  if (wrapped > pi<T>()) {
    wrapped -= two_pi<T>();
  } else if (wrapped <= -pi<T>()) {
    wrapped += two_pi<T>();
  }
  return wrapped;
}

/// @brief     ある直線の始点と終点が与えられたとき,終点から左右に任意の長さ分ずらした2点を返す
/// @param begin ある直線についての始点
/// @param end ある直線についての終点
/// @param shift 終点からずらしたい長さ
/// @return
/// std::tuple<Eigen::Vector2d,Eigen::Vector2d>{終点から右にずらした点,終点から左にずらした点}
template <class T>
auto move(Eigen::Vector2d begin, Eigen::Vector2d end, T shift) {
  //移動した量
  const Eigen::Vector2d move{begin};

  //計算の為に中心にずらした場合の座標
  Eigen::Vector2d after_begin{begin - move};

  Eigen::Vector2d after_end{end - move};

  // x軸から角度
  const auto alpha = util::wrap_to_2pi(
      std::atan2(after_end.y() - after_begin.y(), after_end.x() - after_begin.x()));
  //回転行列
  const Eigen::Rotation2Dd rotate(alpha);

  after_end.x() = (after_end - after_begin).norm();
  after_end.y() = 0.0;

  //移動した先での仮の座標
  const Eigen::Vector2d tmp1(after_end.x(), shift);
  const Eigen::Vector2d tmp2(tmp1.x(), tmp1.y() * (-1));

  std::tuple<Eigen::Vector2d, Eigen::Vector2d> t =
      std::make_tuple((rotate * tmp1) + move, (rotate * tmp2) + move);
  //回転した後の正しい座標
  return t;
}
}
}
#endif
