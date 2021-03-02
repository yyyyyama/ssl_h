#ifndef AI_SERVER_UTIL_MATH_ANGLE_H
#define AI_SERVER_UTIL_MATH_ANGLE_H

#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <complex>
#include <iterator>
#include <numeric>

#include "detail/xyz.h"

namespace ai_server {
namespace util {
namespace math {

template <class T, std::enable_if_t<std::is_floating_point<T>::value, std::nullptr_t> = nullptr>
T wrap_to_2pi(T r) {
  using boost::math::constants::two_pi;

  auto wrapped = std::fmod(r, two_pi<T>());

  if (r < 0) {
    wrapped += two_pi<T>();
  }
  return wrapped;
}

/// @brief	-pi<r<=piに正規化
template <class T, std::enable_if_t<std::is_floating_point<T>::value, std::nullptr_t> = nullptr>
T wrap_to_pi(T r) {
  using boost::math::constants::pi;
  using boost::math::constants::two_pi;

  auto wrapped = std::fmod(r, two_pi<T>());

  if (wrapped > pi<T>()) {
    wrapped -= two_pi<T>();
  } else if (wrapped <= -pi<T>()) {
    wrapped += two_pi<T>();
  }
  return wrapped;
}

/// @brief  物理的な視点から，角度fromを基準にした角度toとの差分をとる
/// @param    from 基準とする角度。値の範囲は問わない。
/// @param    to   差分をとる対象の角度。値の範囲は問わない。
/// @return  角度差。値の範囲は [-pi, pi]
template <class T, std::enable_if_t<std::is_floating_point_v<T>, std::nullptr_t> = nullptr>
inline auto delta_theta(T from, T to) {
  // 内積: cos(to) * cos(from) + sin(to) * sin(from) = cos(to-from)
  const auto dot = std::cos(to - from);
  // 外積のz: sin(to) * cos(from) -  cos(to) * sin(from) = sin(to-from)
  const auto rot_z = std::sin(to - from);
  // 外積で角度差の符号を，内積で角度差の絶対値を導出し，atan2をとる。
  return std::atan2(rot_z, dot);
}

/// @brief   2次元ベクトルの方向を返す
/// @param v   2次元ベクトル
/// @return  2次元ベクトルの方向。値の範囲は [-pi, pi]
template <class T>
inline auto direction(const T& v) -> decltype(std::atan2(detail::y(v), detail::x(v))) {
  return std::atan2(detail::y(v), detail::x(v));
}

/// @brief   2次元ベクトルにおいて、始点からみた終点の方向を返す
/// @param v_end    2次元ベクトルの終点
/// @param v_init   2次元ベクトルの始点
/// @return  始点からみた終点の方向。値の範囲は [-pi, pi]
template <class T, class U>
inline auto direction(const T& v_end, const U& v_init)
    -> decltype(std::atan2(detail::y(v_end) - detail::y(v_init),
                           detail::x(v_end) - detail::x(v_init))) {
  return std::atan2(detail::y(v_end) - detail::y(v_init), detail::x(v_end) - detail::x(v_init));
}

/// @brief 180度回転させた角度を返す
/// @param    theta   角度
/// @return  180度回転させた角度。値の範囲は [-pi, pi]
template <class T, std::enable_if_t<std::is_floating_point_v<T>, std::nullptr_t> = nullptr>
inline auto inverse(T theta) {
  return wrap_to_pi(theta + boost::math::constants::pi<T>());
}

//  @brief  角度の平均を求める
/// @param    first 対象とする角度集合の範囲
/// @param    last  対象とする角度集合の範囲
/// @return  角度平均。値の範囲は [-pi, pi]
template <class Iterator,
          std::enable_if_t<
              std::is_floating_point_v<typename std::iterator_traits<Iterator>::value_type>,
              std::nullptr_t> = nullptr>
inline auto theta_average(Iterator first, Iterator last) {
  using value_t = typename std::iterator_traits<Iterator>::value_type;

  // 角度を単位ベクトルに見立て，その総和をとる
  const auto sum =
      std::accumulate(first, last, std::complex<value_t>{},
                      [](const auto& a, const auto& b) { return a + std::polar(1.0, b); });

  // 平均 = 総和ベクトルの角度
  return std::arg(sum);
}

/// @brief 偏角xが偏角yの左側に見えるか
/// @param   x 偏角
/// @param   y 基準となる偏角
/// @return wrap_to_pi(x-y) > 0
template <class T, std::enable_if_t<std::is_floating_point<T>::value, std::nullptr_t> = nullptr>
inline bool left_of(T x, T y) {
  return wrap_to_pi(x - y) > T();
}

/// @brief 偏角xが偏角yの右側に見えるか
/// @param   x 偏角
/// @param   y 基準となる偏角
/// @return wrap_to_pi(x-y) < 0
template <class T, std::enable_if_t<std::is_floating_point<T>::value, std::nullptr_t> = nullptr>
inline bool right_of(T x, T y) {
  return wrap_to_pi(x - y) < T();
}

} // namespace math
} // namespace util
} // namespace ai_server
#endif
