#ifndef AI_SERVER_UTIL_MATH_DETAIL_DIRECTION_H
#define AI_SERVER_UTIL_MATH_DETAIL_DIRECTION_H

#include <cmath>
#include <type_traits>

namespace ai_server::util::math::detail {

/// @brief ベクトル1から見たベクトル2の角度を返す
/// @param x2   ベクトル2の x
/// @param y2   ベクトル2の y
/// @param x1   ベクトル1の x
/// @param y1   ベクトル1の y
template <class T, std::enable_if_t<std::is_floating_point_v<T>, std::nullptr_t> = nullptr>
inline auto direction_from(T x2, T y2, T x1, T y1) {
  // 内積
  const auto dot = x1 * x2 + y1 * y2;
  // 外積のz
  const auto rot_z = x1 * y2 - x2 * y1;

  return std::atan2(rot_z, dot);
}
} // namespace ai_server::util::math::detail

#endif
