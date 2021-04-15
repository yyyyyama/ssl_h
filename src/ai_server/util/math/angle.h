#ifndef AI_SERVER_UTIL_MATH_ANGLE_H
#define AI_SERVER_UTIL_MATH_ANGLE_H

#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <complex>
#include <iterator>
#include <numeric>

#include "detail/direction.h"
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

/// @brief 基準との角度差をとる。
/// @param    arg   角度
/// @param    ref   基準の角度
/// @return  wrap_to_pi(arg - ref)
template <class T, std::enable_if_t<std::is_floating_point_v<T>, std::nullptr_t> = nullptr>
inline auto direction_from(T arg, T ref) {
  return wrap_to_pi(arg - ref);
}

/// @brief xy空間において、基準ベクトルとの角度差をとる。
/// @param    v     ベクトル
/// @param    ref   基準ベクトル
/// @return  wrap_to_pi(direction(v) - direction(ref)) に等しい値
template <class T, class U>
inline auto direction_from(const T& v, const U& ref)
    -> decltype(detail::direction_from(detail::x(v), detail::y(v), detail::x(ref),
                                       detail::y(ref))) {
  return detail::direction_from(detail::x(v), detail::y(v), detail::x(ref), detail::y(ref));
}

/// @brief xy空間において、基準ベクトルとの角度差をとる。
/// @param    v_end      ベクトルの終点
/// @param    v_init     ベクトルの始点
/// @param    ref_end    基準ベクトルの終点
/// @param    ref_init   基準ベクトルの始点
/// @return  wrap_to_pi(direction(v_end, v_init) - direction(ref_end, ref_init)) に等しい値
template <class T, class U, class V, class W>
inline auto direction_from(const T& v_end, const U& v_init, const V& ref_end, const W& ref_init)
    -> decltype(detail::direction_from(detail::x(v_end) - detail::x(v_init),
                                       detail::y(v_end) - detail::y(v_init),
                                       detail::x(ref_end) - detail::x(ref_init),
                                       detail::y(ref_end) - detail::y(ref_init))) {
  return detail::direction_from(
      detail::x(v_end) - detail::x(v_init), detail::y(v_end) - detail::y(v_init),
      detail::x(ref_end) - detail::x(ref_init), detail::y(ref_end) - detail::y(ref_init));
}

/// @brief xy空間において、小さい方の角度差をとり、その絶対値を返す
/// @param    arg   角度
/// @param    ref   基準の角度
/// @return  abs(wrap_to_pi(arg - ref))
template <class T, std::enable_if_t<std::is_floating_point_v<T>, std::nullptr_t> = nullptr>
inline auto inferior_angle(T arg, T ref) {
  return std::abs(wrap_to_pi(arg - ref));
}

/// @brief xy空間において、小さい方の角度差をとり、その絶対値を返す
/// @param    v     ベクトル
/// @param    ref   基準ベクトル
/// @return  abs(wrap_to_pi(direction(v) - direction(ref))) に等しい値。
template <class T, class U>
inline auto inferior_angle(const T& v, const U& ref)
    -> decltype(detail::direction_from(detail::x(v), detail::y(v), detail::x(ref),
                                       detail::y(ref))) {
  return std::abs(
      detail::direction_from(detail::x(v), detail::y(v), detail::x(ref), detail::y(ref)));
}

/// @brief xy空間において、小さい方の角度差をとり、その絶対値を返す
/// @param    v_end      ベクトルの終点
/// @param    v_init     ベクトルの始点
/// @param    ref_end    基準ベクトルの終点
/// @param    ref_init   基準ベクトルの始点
/// @return  abs(wrap_to_pi(direction(v_end, v_init) - direction(ref_end, ref_init))) に等しい値
template <class T, class U, class V, class W>
inline auto inferior_angle(const T& v_end, const U& v_init, const V& ref_end, const W& ref_init)
    -> decltype(detail::direction_from(detail::x(v_end) - detail::x(v_init),
                                       detail::y(v_end) - detail::y(v_init),
                                       detail::x(ref_end) - detail::x(ref_init),
                                       detail::y(ref_end) - detail::y(ref_init))) {
  return std::abs(detail::direction_from(
      detail::x(v_end) - detail::x(v_init), detail::y(v_end) - detail::y(v_init),
      detail::x(ref_end) - detail::x(ref_init), detail::y(ref_end) - detail::y(ref_init)));
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
