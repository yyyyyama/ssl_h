#ifndef AI_SERVER_UTIL_MATH_DISTANCE_H
#define AI_SERVER_UTIL_MATH_DISTANCE_H

#include <cmath>

#include "detail/xyz.h"

namespace ai_server::util::math {

/// @brief   xy平面における2点の距離を求める
/// @param a 点1の座標
/// @param b 点2の座標
/// @return  floatまたはdouble
template <class T, class U>
inline auto distance(const T& a, const U& b)
    -> decltype(std::hypot(detail::x(a) - detail::x(b), detail::y(a) - detail::y(b))) {
  return std::hypot(detail::x(a) - detail::x(b), detail::y(a) - detail::y(b));
}

/// @brief   xyz空間における2点の距離を求める
/// @param a 点1の座標
/// @param b 点2の座標
/// @return  floatまたはdouble
template <class T, class U>
inline auto distance3d(const T& a, const U& b)
    -> decltype(std::hypot(detail::x(a) - detail::x(b), detail::y(a) - detail::y(b),
                           detail::z(a) - detail::z(b))) {
  return std::hypot(detail::x(a) - detail::x(b), detail::y(a) - detail::y(b),
                    detail::z(a) - detail::z(b));
}
} // namespace ai_server::util::math

#endif
